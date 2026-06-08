from controllers.crazyfliepy import trayectoria
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
        self.kp = 0.0
        self.kp_roll=0.0
        self.kp_pitch=0.0
        self.kp_yaw=0.0
        self.kp_vx=0.0
        self.kp_vy=0.0
        #Kd
        self.kd = 0.0
        self.kd_roll=0.0
        self.kd_pitch=0.0
        self.kd_yaw=0.0
        self.kd_vx=0.0
        self.kd_vy=0.0
        #ki
        self.ki=0.0
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

        self.altura=1.0
       

    def control_altitud(self,altitud_act,dt):
        error_actual=self.altura-altitud_act
        error_derivado=(error_actual-self.error_altitud)/dt
        self.integrador_altitud=self.integrador_altitud+error_actual*dt
        error_actual=clamp(error_actual,-1,1)
        comando_z=((error_actual*self.kp)+(error_derivado*self.kd)+(self.integrador_altitud*self.ki))+self.k
        self.error_altitud=error_actual
        return comando_z
    
    def control_roll(self,roll_act,dt):
        pass

    def control_pitch(self,pitch_act,dt):
        pass 

    def control_yaw(self,yaw_act,dt):
        pass 






        

        
        

        



