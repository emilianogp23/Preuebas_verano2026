"""Controlador_principal controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
# pyrefly: ignore [missing-import]
from controller import Robot
from pid import PID
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



pid_obj=PID()
odom_obj=OdometriaIMU()
tray_obj=control_trayectoria()

num_puntos=tray_obj.puntos_necesarios()
puntos=tray_obj.puntos_trayectoria_circular(num_puntos)

timepo_pas=robot.getTime()





# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:

    timepo_act=robot.getTime()
    dt=timepo_act-timepo_pas
    timepo_pas=timepo_act
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

    odom=odom_obj.update(dt, [ax,ay,az], [roll,pitch,yaw])
    pos_act=odom[0]
    vel_act=odom[1]

    error=pos_act-puntos[0]
    
    



    print(odom)
    print(puntos[0])



    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
