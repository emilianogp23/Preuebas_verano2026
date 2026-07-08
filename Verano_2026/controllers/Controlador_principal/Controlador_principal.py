#Controlador_principal controller
from pid import normalizar_angulos
# pyrefly: ignore [missing-import]
from controller import Supervisor#Robot
import numpy as np 
import math as mt
import scipy.interpolate as spi
from pid import PID
# pyrefly: ignore [missing-import]
from odometria_imu import OdometriaIMU
#from trayectoria import control_trayectoria
from generador_tr import Generador
import json
import sys
import os 
import glob

def apagar_motores(m1,m2,m3,m4):
    m1.setVelocity(0.0)
    m2.setVelocity(0.0)
    m3.setVelocity(0.0)
    m4.setVelocity(0.0)


# create the Robot instance.
robot = Supervisor()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
m1=robot.getDevice('m1_motor')
m1.setPosition(float('inf'))
m1.setVelocity(0.0)

m2=robot.getDevice('m2_motor')
m2.setPosition(float('inf'))
m2.setVelocity(0.0)

m3=robot.getDevice('m3_motor')
m3.setPosition(float('inf'))
m3.setVelocity(0.0)

m4=robot.getDevice('m4_motor')
m4.setPosition(float('inf'))
m4.setVelocity(0.0)

imu=robot.getDevice('inertial_unit')
imu.enable(timestep)

gps=robot.getDevice('gps')
gps.enable(timestep)

gyro=robot.getDevice('gyro')
gyro.enable(timestep)

camera =robot.getDevice('camera')
camera.enable(timestep)

acelerometro=robot.getDevice('accelerometer')
acelerometro.enable(timestep)


#Ganancias para convertir de distancia a velocidad 
kp_vx=0.4  #0.3
kp_vy=0.4

pid_obj=PID()
# odom_obj=OdometriaIMU()
# tray_obj=control_trayectoria()
# gen_obj=Generador()

# num_puntos=tray_obj.puntos_necesarios()
# puntos=tray_obj.puntos_trayectoria_circular(num_puntos)
#waypoints=tray_obj.puntos_trayectoria_circular(num_puntos)
# Obtener rutas dinámicas
dir_controlador = os.path.dirname(os.path.abspath(__file__))
ruta_base = os.path.abspath(os.path.join(dir_controlador, "..", ".."))
ruta = os.path.join(ruta_base, "mision.json")
ruta2 = os.path.join(ruta_base, "reporte_vuelo.json")
ruta_resultados=os.path.join(ruta_base, "Resultados")
if not os.path.exists(ruta_resultados):
    os.makedirs(ruta_resultados)

ruta_fotos = os.path.join(ruta_resultados, "fotos_capturadas")
ruta_debug = os.path.join(ruta_resultados, "fotos_debug")

try:
    with open(ruta, 'r') as f:
        datos=json.load(f)
        waypoints = datos["waypoints"]
except:
    waypoints=[[0.0,0.0,0.0]]
    print("No se encontraron waypoints")
    sys.exit()

puntos=waypoints

indice_wp=0

# #separar coordenadas de puntos 
# xvec=[p[0] for p in puntos]
# yvec=[p[1] for p in puntos]

vel=1.0    #v=d/t  t=d/v

#posicion del objeto en mapa
# pos_x=1.0
# pos_y=1.0

# xvec_orig=xvec
# yvec_orig=yvec
# thvec_orig=[mt.atan2(pos_y-y,pos_x-x) for x , y in zip(xvec_orig,yvec_orig)]

# tiempo_por_punto = 5.0
# tvec = [i * tiempo_por_punto for i in range(len(xvec_orig) + 1)]


#t_inicial=robot.getTime()
funcionando=False
t_inicial=0.0

# Esperar 2 segundos para inicializar sensores en Webots
while robot.step(timestep) != -1:
    if robot.getTime() > 2.0:
        break

timepo_pas=robot.getTime()
past_x=gps.getValues()[0]
past_y=gps.getValues()[1]
past_z=gps.getValues()[2]

#variables para trayectoria 4D
t_vuelo=0.0
a0,a1,a2,a3=None,None,None,None
vel_dron=vel#0.8 #m/s

os.makedirs(ruta_fotos, exist_ok=True)

archivos_anteriores = glob.glob(os.path.join(ruta_fotos, "*"))
for archivo in archivos_anteriores:
    os.remove(archivo)
print("Carpeta limpia ")

#Variables para captura de fotos
contador_fotos=0
posicion_inicial_real=[past_x,past_y,past_z]
ps_ini=[past_x,past_y,past_z]
espacio_fotos=1.0 #metros




while robot.step(timestep) != -1:

    t_act=robot.getTime()
    dt=t_act-timepo_pas
    timepo_pas=t_act
    roll=imu.getRollPitchYaw()[0]
    pitch=imu.getRollPitchYaw()[1]
    yaw=imu.getRollPitchYaw()[2]
    
    x=gps.getValues()[0]
    y=gps.getValues()[1]
    z=gps.getValues()[2]
    dx=1.0
    dy=1.0
    yd=mt.atan2(dy-y,dx-x)

    giroscopio=gyro.getValues()
    ax=acelerometro.getValues()[0]
    ay=acelerometro.getValues()[1]
    az=acelerometro.getValues()[2]

    wx=giroscopio[0]
    wy=giroscopio[1]
    wz=giroscopio[2]

    #odom=odom_obj.update(dt, [ax,ay,az], [roll,pitch,yaw])  #cmbios de prueba
    pos_act=[x,y,z]#odom[0]

    #vel_act=odom[1]  #cmbios de prueba

    dist_recorrida=mt.sqrt((ps_ini[0]-x)**2 + (ps_ini[1]-y)**2 + (ps_ini[2]-z)**2)
    if dist_recorrida>=espacio_fotos:
        camera.saveImage(os.path.join(ruta_fotos, f"foto_{contador_fotos:04d}.png"), 100)
        print(f"Foto {contador_fotos} guardada")
        contador_fotos += 1
        ps_ini=[x,y,z]
        dist_recorrida=0.0  
    
    if indice_wp < len(puntos):
        
        if funcionando == False:
            from scipy.interpolate import CubicSpline
            
            # 1. Creamos la lista de puntos iniciando desde la posición ACTUAL del dron
            distancias = [0.0]
            puntos_limpios = [pos_act] 
            
            # 2. Unimos todos los waypoints midiendo la distancia entre ellos
            for i in range(len(puntos)):
                p_prev = puntos_limpios[-1]
                p_act = puntos[i][:3]  # Asegurar que solo tomamos X,Y,Z
                
                # Calculamos la distancia con el punto anterior
                d = mt.sqrt((p_act[0]-p_prev[0])**2 + (p_act[1]-p_prev[1])**2 + (p_act[2]-p_prev[2])**2)
                
                # Evitamos duplicados o puntos demasiado juntos (causan error matemático en el spline)
                if d > 0.01: 
                    distancias.append(distancias[-1] + d)
                    puntos_limpios.append(p_act)
                    
            distancia_total_ruta = distancias[-1]
            
            # Tiempo estimado para recorrer TODO el circuito a velocidad constante
            t_vuelo = max(distancia_total_ruta / vel_dron, 1.0)
            
            # 3. Generar el Spline Cúbico Continuo (la curva suave)
            # Esto interpola todos los puntos en un solo trazo sin frenadas
            spline_trayectoria = CubicSpline(distancias, puntos_limpios)
            
            t_inicial = t_act
            funcionando = True
        
        # 4. Calcular dónde debería estar el dron en este instante de tiempo
        t_tray = t_act - t_inicial
        
        # Distancia teórica que el dron debió haber recorrido a velocidad constante
        distancia_actual = min(t_tray * vel_dron, distancia_total_ruta)
        
        # Evaluar la curva en esa distancia
        pos_des = spline_trayectoria(distancia_actual)
        x_des = float(pos_des[0])
        y_des = float(pos_des[1])
        z_des = float(pos_des[2])
        w_des = normalizar_angulos(yd) 
        
        # 5. Condición para terminar la simulación (cuando el tiempo total se acaba)
        # Le damos 1.0 segundo extra para asegurar que llegó al último punto
        if t_tray > t_vuelo + 1.0: 
            indice_wp = len(puntos) # Esto forzará que pase al 'else' de abajo y apague motores
    else:
        apagar_motores(m1,m2,m3,m4)
        reporte_vuelo={
            "t_vuelo":t_vuelo,
            "pos_inicial":posicion_inicial_real,
            "pos_final":pos_act,
            "puntos":waypoints,
            "tiempo_total":t_act
        }
        with open(ruta2, "w") as f:
            json.dump(reporte_vuelo, f)
        print("Simulación finalizada")
        robot.simulationQuit(0)
        break


    #errores 
    error_x=x_des-pos_act[0]
    error_y=y_des-pos_act[1]
    error_z=z_des-pos_act[2]
    error_yaw=yaw-w_des

    #error global vx,vy
    vx= max(-1.0, min(1.0, error_x*kp_vx))
    vy= max(-1.0, min(1.0, error_y*kp_vy))

    #error local vx , vy
    vx_local=vx*mt.cos(yaw)+ vy*mt.sin(yaw)
    vy_local=-vx*mt.sin(yaw)+ vy*mt.cos(yaw)

    #Control en cascada 

    #Variables para control 
    # vx_act=vel_act[0]   #cmbios de prueba
    # vy_act=vel_act[1]  #cmbios de prueba

    vx_global_act=(x-past_x)/dt
    vy_global_act=(y-past_y)/dt

    vx_act=vx_global_act*mt.cos(yaw)+ vy_global_act*mt.sin(yaw)
    vy_act=-vx_global_act*mt.sin(yaw)+ vy_global_act*mt.cos(yaw)

    past_x=x
    past_y=y

    vx_des=vx_local
    vy_des=vy_local
    pitch_act=pitch
    roll_act=roll
    #yaw_des=w_des - mt.pi
    yaw_des=normalizar_angulos(w_des)

    #1.Lazo externo 
    pitch_des=pid_obj.control_vx(vx_act,vx_des,dt)
    roll_des=pid_obj.control_vy(vy_act,vy_des,dt)

    #2. Lazo interno 
    com_x=pid_obj.control_pitch(pitch_des,pitch_act,dt)
    com_y=pid_obj.control_roll(roll_des,roll_act,dt)
    com_yaw=pid_obj.control_yaw(yaw_des,yaw,dt,wz)

    #3.Altitud y yaw 
    com_z=pid_obj.control_altitud(z_des, z, dt)

    #4.Mezcla de motores
    vel_m1,vel_m2,vel_m3,vel_m4=pid_obj.vel_motores(com_y,com_x,com_yaw,com_z)
    m1.setVelocity(-vel_m1)
    m2.setVelocity(vel_m2)
    m3.setVelocity(-vel_m3)
    m4.setVelocity(vel_m4)
    