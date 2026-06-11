"""Controlador_principal controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
# pyrefly: ignore [missing-import]
from controller import Robot
import numpy as np 
import math as mt
import scipy.interpolate as spi
from pid import PID
# pyrefly: ignore [missing-import]
from odometria_imu import OdometriaIMU
from trayectoria import control_trayectoria


# create the Robot instance.
robot = Robot()

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
kp_vx=0.3  #0.3
kp_vy=0.3

pid_obj=PID()
odom_obj=OdometriaIMU()
tray_obj=control_trayectoria()

num_puntos=tray_obj.puntos_necesarios()
puntos=tray_obj.puntos_trayectoria_circular(num_puntos)

timepo_pas=robot.getTime()

#separar coordenadas de puntos 
xvec=[p[0] for p in puntos]
yvec=[p[1] for p in puntos]

vel=0.5    #v=d/t  t=d/v

#posicion del objeto en mapa
pos_x=1.0
pos_y=1.0



tvec=[0.0]
thvec=[mt.atan2(pos_y-y,pos_x-x) for x , y in zip(xvec,yvec)]
thvec=np.unwrap(thvec).tolist()

for i in range(len(puntos)-1):
    d=mt.dist([xvec[i],yvec[i]],[xvec[i+1],yvec[i+1]])
    t=(d/vel)
    timepo_acumulado=tvec[-1]+t
    tvec.append(timepo_acumulado)

interpol_x=spi.splrep(tvec,xvec)
interpol_y=spi.splrep(tvec,yvec)
interpol_w=spi.splrep(tvec,thvec)

#t_inicial=robot.getTime()
funcionando=False
t_inicial=0.0
past_x=0.0
past_y=0.0

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

    if funcionando==False:
        x_des=puntos[0][0]
        y_des=puntos[0][1]
        w_des=thvec[0]
        d_ini=mt.dist([pos_act[0],pos_act[1]],[x_des,y_des])

        if d_ini< 0.20:
            funcionando=True
            t_inicial=t_act
    else:
        tcurr=t_act-t_inicial

        if tcurr > tvec[-1]:
            tcurr=tvec[-1]
        
        x_des=spi.splev(tcurr,interpol_x)
        y_des=spi.splev(tcurr,interpol_y)
        w_des=spi.splev(tcurr,interpol_w)

    #errores 
    error_x=x_des-pos_act[0]
    error_y=y_des-pos_act[1]
    error_z=tray_obj.altura_vuelo-pos_act[2]
    error_yaw=yaw-w_des

    #error global vx,vy
    vx= error_x*kp_vx
    vy=error_y*kp_vy

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
    yaw_des=w_des

    

    #1.Lazo externo 
    pitch_des=pid_obj.control_vx(vx_act,vx_des,dt)
    roll_des=pid_obj.control_vy(vy_act,vy_des,dt)

    #2. Lazo interno 
    com_x=pid_obj.control_pitch(pitch_des,pitch_act,dt)
    com_y=pid_obj.control_roll(roll_des,roll_act,dt)
    com_yaw=pid_obj.control_yaw(yaw_des,yaw,dt)

    #3.Altitud y yaw 
    com_z=pid_obj.control_altitud(z,dt)



    #4.Mezcla de motores
    vel_m1,vel_m2,vel_m3,vel_m4=pid_obj.vel_motores(com_y,com_x,com_yaw,com_z)
    m1.setVelocity(-vel_m1)
    m2.setVelocity(vel_m2)
    m3.setVelocity(-vel_m3)
    m4.setVelocity(vel_m4)
    

    #rint(odom)
    print(f"x: {pos_act[0]}, y: {pos_act[1]}, z: {pos_act[2]}")
    print(f"vx_act: {vx_act}, vy_act: {vy_act}")
    print(f"vx_des: {vx_des}, vy_des: {vy_des}")
    print(puntos[0])

# Enter here exit cleanup code.
