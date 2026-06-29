import math
from controller import Robot, Camera, DistanceSensor, GPS, Gyro, InertialUnit, Keyboard, Motor
import scipy.interpolate as spi
import cv2
import numpy as np
from ultralytics import YOLO

# Añadir el controlador externo - pid_controller.py
from pid_controller import (
    ActualState, #Estado actual
    DesiredState, #Estado deseado
    GainsPID, # Ganancias del PID
    MotorPower, #Potencia del motor
    init_pid_attitude_fixed_height_controller,
    pid_velocity_fixed_height_controller,
)

FLYING_ALTITUDE = 0.6 #Estable la altura fija del drone - 1m

def main():

    robot = Robot()

    timestep = int(robot.getBasicTimeStep())
    
    #Inicia los motores
    m1_motor = robot.getDevice("m1_motor")
    m1_motor.setPosition(float("inf"))
    m1_motor.setVelocity(-1.0)

    m2_motor = robot.getDevice("m2_motor")
    m2_motor.setPosition(float("inf"))
    m2_motor.setVelocity(1.0)

    m3_motor = robot.getDevice("m3_motor")
    m3_motor.setPosition(float("inf"))
    m3_motor.setVelocity(-1.0)

    m4_motor = robot.getDevice("m4_motor")
    m4_motor.setPosition(float("inf"))
    m4_motor.setVelocity(1.0)

    # Inicializar los sensores
    imu = robot.getDevice("inertial_unit")
    imu.enable(timestep)

    gps = robot.getDevice("gps")
    gps.enable(timestep)

    keyboard = robot.getKeyboard() #Debido a que    
    keyboard.enable(timestep)

    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)

    camera = robot.getDevice("camera") # El que me interesa por ahora
    camera.enable(timestep)

    # Permiten conocer las distancias de los obstaculos
    range_front = robot.getDevice("range_front")
    range_front.enable(timestep)

    range_left = robot.getDevice("range_left")
    range_left.enable(timestep)

    range_back = robot.getDevice("range_back")
    range_back.enable(timestep)

    range_right = robot.getDevice("range_right")
    range_right.enable(timestep)
    ###################################################

    # Espera por dos segundos - El tiempo es necesario para estabilizar la simulacion y los sensores.
    while robot.step(timestep) != -1:
        if robot.getTime() > 2.0:
            break

    # Iniciacion de las variables 
    actual_state = ActualState()
    desired_state = DesiredState()
    past_x_global = gps.getValues()[0] #Obtiene la posicion global en x del drone - De esta manera el dron se puede posicionar en cualquier parte del mapa sin afectar su funcionamiento basico.
    past_y_global = gps.getValues()[1] #Obtiene la posicion global en y del drone - De esta manera el dron se puede posicionar en cualquier parte del mapa sin afectar su funcionamiento basico.
    past_time = robot.getTime()

    # Iniciacion de los valores (gains) para el controlador PID
    gains_pid = GainsPID()
    gains_pid.kp_att_y = 1.0
    gains_pid.kd_att_y = 0.5
    gains_pid.kp_att_rp = 0.5
    gains_pid.kd_att_rp = 0.1
    gains_pid.kp_vel_xy = 2.0
    gains_pid.kd_vel_xy = 0.5
    gains_pid.kp_z = 10.0
    gains_pid.ki_z = 5.0
    gains_pid.kd_z = 5.0
    init_pid_attitude_fixed_height_controller()

    height_desired = FLYING_ALTITUDE

    # Inicializar la estructura para la potencia del motor
    motor_power = MotorPower()

    print()
    print("====== Controls =======")
    print(" The Crazyflie can be controlled from your keyboard!")
    print(" All controllable movement is in body coordinates")
    print("- Use the up, back, right and left button to move in the horizontal plane")
    print("- Use Q and E to rotate around yaw")
    print("- Use W and S to go up and down")

    Kp = 5
    xvec = [0, 1, -1]
    yvec = [0, 1, 1]
    thvec = [math.pi/4, math.pi/2, math.pi]
    tvec = [0, 5, 10, 15]

    stable = False

    #Variables para el sistema de navegacion por waypoints
    current_wp_index = 0
    waypoints = [] #list para guardar los waypoints
    th0 = 0.0 #Valor inicial del angulo de yaw 
    smoothed_vx = 0.0 #Velocidad en X del dron (promedio de las ultimas velocidades)
    smoothed_vy = 0.0 #Velocidad en Y del dron (promedio de las ultimas velocidades)

    tini = robot.getTime()

    # Cargar modelo YOLO
    model = YOLO("yolov8n.pt")
    cv2.namedWindow("Drone Camera", cv2.WINDOW_AUTOSIZE)

    while robot.step(timestep) != -1:
        dt = robot.getTime() - past_time

        # Obtiene las mediciones
        rpy = imu.getRollPitchYaw()
        actual_state.roll = rpy[0]
        actual_state.pitch = rpy[1]
        actual_state.yaw_rate = gyro.getValues()[2]
        actual_state.altitude = gps.getValues()[2]

        x_global = gps.getValues()[0]
        vx_global = (x_global - past_x_global) / dt
        y_global = gps.getValues()[1]
        vy_global = (y_global - past_y_global) / dt

        # Obtener velocidades fijas del cuerpo
        actual_yaw = imu.getRollPitchYaw()[2]
        cos_yaw = math.cos(actual_yaw)
        sin_yaw = math.sin(actual_yaw)
        actual_state.vx = vx_global * cos_yaw + vy_global * sin_yaw
        actual_state.vy = -vx_global * sin_yaw + vy_global * cos_yaw

        # Iniciacion los valores de estado deseados
        desired_state.roll = 0.0
        desired_state.pitch = 0.0
        desired_state.vx = 0.0
        desired_state.vy = 0.0
        desired_state.yaw_rate = 0.0
        desired_state.altitude = 1.0

        forward_desired = 0.0
        sideways_desired = 0.0
        yaw_desired = 0.0
        height_diff_desired = 0.0

        '''# Control del drone mediante el teclado
        key = keyboard.getKey()
        while key > 0:
            if key == Keyboard.UP:
                forward_desired = +0.5
            elif key == Keyboard.DOWN:
                forward_desired = -0.5
            elif key == Keyboard.RIGHT:
                sideways_desired = -0.5
            elif key == Keyboard.LEFT:
                sideways_desired = +0.5
            elif key == ord("Q"):
                yaw_desired = 1.0
            elif key == ord("E"):
                yaw_desired = -1.0
            elif key == ord("W"):
                height_diff_desired = 0.1
            elif key == ord("S"):
                height_diff_desired = -0.1

            key = keyboard.getKey()
        tcurr = robot.getTime()-tini
        '''
        
        '''## Movimientos a velocidad constante
        theta = 2*math.pi*tcurr/10.0
        v = 0.5
        forward_desired = v*math.cos(theta)
        sideways_desired = v*math.sin(theta)
        if tcurr > 5:
            height_diff_desired = 0.15*math.sin(2*math.pi*3*tcurr)
        yaw_desired = 0.0
        '''
        
        '''## Controlador tipo robot DDR
         Kv = 0.5
         Kh = 1.0
         xd = 0
         yd = 0
         errp = math.sqrt((x_global-xd)**2 + (y_global-yd)**2)
         ct = math.cos(rpy[2])
         st = math.sin(rpy[2])
         errh = math.atan2(ct*(yd-y_global)-st*(xd-x_global), st*(yd-y_global)+ct*(xd-x_global))
         forward_desired = Kv*errp
         yaw_desired = Kh*errh
        '''

        # Controlador para la orientación o yaw - Segmento que se encarga de la navegacion automatica
        Kh = 5.0
        th = rpy[2]

        '''
        if robot.getTime() > 5:
            if stable == False:
                #inicializa interp
                xvec = [x_global] + xvec
                yvec = [y_global] + yvec
                thvec = [th] + thvec
                xc = spi.splrep(tvec, xvec)
                yc = spi.splrep(tvec, yvec)
                thc = spi.splrep(tvec, thvec)
                tini = robot.getTime()
                stable = True
            else:
                tcurr = robot.getTime()-tini
                xd = spi.splev(tcurr, xc)
                yd = spi.splev(tcurr, yc)
                vx = -Kp*(x_global-xd)
                vy = -Kp*(y_global-yd)
                forward_desired = math.cos(th)*vx+math.sin(th)*vy
                sideways_desired = -math.sin(th)*vx+math.cos(th)*vy
                height_desired = 0.6 #Altura deseada del dron para el sistema automatico de rutas
                thd = spi.splev(tcurr, thc)
                dang = math.fmod(th-thd, 2.0*math.pi)
                yaw_desired = -Kh*dang
                if tcurr > tvec[-1]:
                    forward_desired = 0
                    sideways_desired = 0
                    yaw_desired = 0
        else:
            height_desired += height_diff_desired * dt
        '''

        # Sistema de navegacion basado en waypoint - se configuro una ruta por defecto
        if robot.getTime() > 5:
            if stable == False:
                x0 = x_global
                y0 = y_global
                th0 = th
                # El dron avanza hacia +Y. Evadir derecha = +X, Evadir izquierda = -X
                waypoints = [
                    (x0, y0 + 1.5),       # WP1: Avanza de frente hacia el Obs 1
                    (x0 + 1, y0 + 1.5),   # WP2: Evasión derecha (+X) manteniendo el avance (+1.5 en Y)
                    (x0 + 1, y0 + 3.2),   # WP3: Avanza pasando el Obs 1
                    (x0, y0 + 3.2),       # WP4: Vuelve al centro
                    (x0, y0 + 4.9),       # WP5: Avanza de frente hacia el Obs 2
                    (x0 - 1, y0 + 4.9),   # WP6: Evasión izquierda (-X) manteniendo el avance (+3.9 en Y)
                    (x0 - 1, y0 + 6.2),   # WP7: Avanza pasando el Obs 2
                    (x0, y0 + 6.2),       # WP8: Vuelve al centro
                    (x0, y0 + 6.61)       # WP9: Posición final
                ]
                current_wp_index = 0
                stable = True
            else:
                if current_wp_index < len(waypoints):
                    xd, yd = waypoints[current_wp_index]
                    dist = math.sqrt((x_global - xd)**2 + (y_global - yd)**2)
                    
                    if dist < 0.15:
                        current_wp_index += 1
                        
                    if current_wp_index < len(waypoints):
                        xd, yd = waypoints[current_wp_index]
                        vx = -Kp * (x_global - xd)
                        vy = -Kp * (y_global - yd)
                        
                        v_mag = math.sqrt(vx**2 + vy**2)
                        # Límite más estricto de velocidad para evitar giros agresivos
                        if v_mag > 0.4:
                            vx = (vx / v_mag) * 0.4
                            vy = (vy / v_mag) * 0.4
                            
                        # Filtro de suavizado (low-pass) para una aceleracion progresiva
                        smoothed_vx = 0.9 * smoothed_vx + 0.1 * vx
                        smoothed_vy = 0.9 * smoothed_vy + 0.1 * vy
                            
                        forward_desired = math.cos(th)*smoothed_vx + math.sin(th)*smoothed_vy
                        sideways_desired = -math.sin(th)*smoothed_vx + math.cos(th)*smoothed_vy
                        
                        dang = th - th0
                        while dang > math.pi: dang -= 2.0 * math.pi
                        while dang < -math.pi: dang += 2.0 * math.pi
                        yaw_desired = -Kh * dang
                    else:
                        forward_desired = 0.0
                        sideways_desired = 0.0
                        yaw_desired = 0.0
                else:
                    forward_desired = 0.0
                    sideways_desired = 0.0
                    yaw_desired = 0.0
                height_desired = 0.6
        else:
            height_desired += height_diff_desired * dt

        # Sistema de reconocimientos de objetos
        img_data = camera.getImage()
        if img_data:
            # Webots devuelve imagen en bytes formato BGRA
            img_width = camera.getWidth()
            img_height = camera.getHeight()
            img_np = np.frombuffer(img_data, np.uint8).reshape((img_height, img_width, 4))
            
            # Convertir a BGR para OpenCV
            img_bgr = cv2.cvtColor(img_np, cv2.COLOR_BGRA2BGR)
            
            # Inferencia YOLO (silenciada para no llenar la consola)
            results = model(img_bgr, verbose=False)
            
            # Obtener el FOV de la cámara
            fov = camera.getFov()
            
            for r in results:
                for box in r.boxes:
                    # Coordenadas Bounding box
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    conf = round(float(box.conf[0]), 2)
                    
                    # Ignorar detecciones con baja confianza (menor al 40%)
                    if conf < 0.4:
                        continue
                        
                    cls = int(box.cls[0])
                    class_name = model.names[cls]
                    
                    # Filtrar por clases (ignorar sillas, mesas, monitores, etc)
                    # "person" es el humano. Las otras son clases típicas con las que YOLO confunde a un barril
                    clases_aceptadas = ["person", "bottle", "cup", "vase", "fire hydrant", "sports ball"]
                    if class_name not in clases_aceptadas:
                        continue
                    
                    # Dibujar bounding box
                    cv2.rectangle(img_bgr, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    
                    # Estimacion de tamaño y distancia
                    bbox_height = float(y2 - y1)
                    if bbox_height <= 0:
                        bbox_height = 1.0 # Prevenir division por cero
                    
                    # Alturas estimadas reales (metros)
                    if class_name == "person":
                        real_height = 1.70 # Humano
                    else:
                        real_height = 1.0 # Barril (YOLO suele detectarlo como bottle, cup, etc)
                        
                    # Calculo de distancia usando Pinhole model
                    dist_estimada = (real_height * img_height) / (bbox_height * 2 * math.tan(fov/2))
                    
                    # Textos a mostrar
                    label = f"{class_name} {conf}"
                    dist_text = f"Dist: {dist_estimada:.2f}m"
                    size_text = f"H_px: {int(bbox_height)}"
                    
                    cv2.putText(img_bgr, label, (int(x1), int(y1)-30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.putText(img_bgr, dist_text, (int(x1), int(y1)-15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                    cv2.putText(img_bgr, size_text, (int(x1), int(y1)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                    
            # Mostrar la ventana
            cv2.imshow("Drone camara", img_bgr)
            cv2.waitKey(1)
        ############################################################

        desired_state.yaw_rate = yaw_desired

        # Controlador de velocidad PID con altura fija
        desired_state.vy = sideways_desired
        desired_state.vx = forward_desired
        desired_state.altitude = height_desired
        pid_velocity_fixed_height_controller(actual_state, desired_state, gains_pid, dt, motor_power)

        # Establecer o configurar las velocidades de los motores
        m1_motor.setVelocity(-motor_power.m1)
        m2_motor.setVelocity(motor_power.m2)
        m3_motor.setVelocity(-motor_power.m3)
        m4_motor.setVelocity(motor_power.m4)

        # Guardar los valores anteriores para el siguiente paso temporal
        past_time = robot.getTime()
        past_x_global = x_global
        past_y_global = y_global


if __name__ == "__main__":
    main()
