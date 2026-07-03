import numpy as np 

def clamp(value, min_value, max_value):
    return max(min_value, min(value, max_value))

def normalizar_angulos(angulo):
    while angulo > np.pi:
        angulo-=2*np.pi
    while angulo < -np.pi:
        angulo+=2*np.pi
    return angulo


class PID:
    def __init__(self):

        self.k=48.0
        #kp
        self.kp = 10.0
        self.kp_roll=0.5
        self.kp_pitch=0.5
        self.kp_yaw=5.0
        self.kp_vx=2.0 #2.0
        self.kp_vy=2.0 #2.0
        #Kd
        self.kd = 5.0
        self.kd_roll=0.1
        self.kd_pitch=0.1
        self.kd_yaw=1.0
        self.kd_vx=0.5
        self.kd_vy=0.5
        #ki
        self.ki=5.0
        self.ki_roll=0.0
        self.ki_pitch=0.0
        self.ki_yaw=0.0
        self.ki_vx=0.0
        self.ki_vy=0.0

        self.error_altitud=0.0
        self.error_pitch=0.0
        self.error_roll=0.0
        self.error_yaw=0.0
        self.error_x=0.0
        self.error_y=0.0
        self.integrador_altitud=0.0

        self.altura=1.3
       

    def control_altitud(self,altitud_des,altitud_act,dt):
        error_actual=altitud_des-altitud_act
        error_derivado=(error_actual-self.error_altitud)/dt
        self.integrador_altitud=self.integrador_altitud+error_actual*dt
        comando_z=(((clamp(error_actual,-1,1))*self.kp)+(error_derivado*self.kd)+(self.integrador_altitud*self.ki))+self.k
        self.error_altitud=error_actual
        return comando_z
    
    def control_roll(self,roll_obj,roll_act,dt):
        error_actual=(roll_obj-roll_act)
        error_derivado=(error_actual-self.error_roll)/dt
        comando_y=(clamp(error_actual,-1,1)*self.kp_roll)+error_derivado*self.kd_roll
        self.error_roll=error_actual
        return comando_y



    def control_pitch(self,pitch_obj,pitch_act,dt):
        error_actual=(pitch_obj-pitch_act)
        error_derivado=(error_actual-self.error_pitch)/dt
        comando_x=(((clamp(error_actual,-1,1))*self.kp_pitch)+(error_derivado*self.kd_pitch))
        self.error_pitch=error_actual
        return comando_x

    def control_yaw(self,yaw_obj,yaw_act,dt, wz):
        error_actual=(yaw_obj-yaw_act)
        error_actual=normalizar_angulos(error_actual)
        # Use gyro (wz) for derivative term (damping)
        # Since wz is yaw_rate (d(yaw)/dt), the derivative of error (yaw_obj - yaw) is -wz
        error_derivado = -wz
        comando_yaw=(((clamp(error_actual,-1,1))*self.kp_yaw)+(error_derivado*self.kd_yaw))
        comando_yaw = clamp(comando_yaw, -1, 1)
        self.error_yaw=error_actual
        return comando_yaw
    
    def control_vx(self,vx_act, vx_des,dt):
        error_actual=(vx_des-vx_act)
        error_derivado=(error_actual-self.error_x)/dt
        comando_pitch=(((clamp(error_actual,-1,1))*self.kp_vx)+(error_derivado*self.kd_vx))
        self.error_x=error_actual
        return comando_pitch
    
    def control_vy(self,vy_act, vy_des,dt):
        error_actual=(vy_des-vy_act)
        error_derivado=(error_actual-self.error_y)/dt
        comando_roll=(((clamp(error_actual,-1,1))*self.kp_vy)+(error_derivado*self.kd_vy))
        self.error_y=error_actual
        return -comando_roll

    def vel_motores(self,roll,pitch,yaw,z):
        vel_m1=z-roll-pitch+yaw#z-roll+pitch+yaw
        vel_m2=z-roll+pitch-yaw#z-roll-pitch-yaw
        vel_m3=z+roll+pitch+yaw#z+roll-pitch+yaw
        vel_m4=z+roll-pitch-yaw#z+roll+pitch-yaw
        velocidades=[vel_m1,vel_m2,vel_m3,vel_m4]
        return velocidades



