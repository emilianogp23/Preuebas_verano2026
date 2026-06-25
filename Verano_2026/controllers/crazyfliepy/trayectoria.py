import numpy as np 
import math as mt 

class control_trayectoria:

    def __init__(self):

        self.fov=0.87
        ladox=1.0  #0.4
        ladoy=1.0
        ladoz=1.0+0.4
        self.altura_vuelo=ladoz/2
        self.perimetro=2*(ladox+ladoy)
        self.area=(ladox*ladoy)
        self.d= 1.0 #radio de trayectoria circular
        self.posx=1.0   #posicion del obj x
        self.posy=1.0   #posicion del obj y
    

    def puntos_necesarios(self):

        
        w=2.0*self.d*mt.tan(self.fov/2.0)
        traslape=0.3
        w_efectivo=w*(1.0-traslape)
        puntos=self.perimetro/w_efectivo
        puntos=mt.ceil(puntos)
        return puntos
    
    def puntos_trayectoria_circular(self,numero_puntos):
        separacion=(2.0*mt.pi)/numero_puntos
        trayectoria=[]
        for n in range(numero_puntos):
            angulo_actual=n*separacion
            xn=(self.d*mt.cos(angulo_actual))+self.posx
            yn=(self.d*mt.sin(angulo_actual))+self.posy
            zn=self.altura_vuelo
            trayectoria.append([xn,yn,zn])
        return trayectoria

if __name__ == "__main__":
    trayectoria=control_trayectoria()
    numero_puntos=trayectoria.puntos_necesarios()
    trayectoria=trayectoria.puntos_trayectoria_circular(numero_puntos)
    for i, punto in enumerate(trayectoria):
        print(f"Punto {i+1}: X={punto[0]:.2f}, Y={punto[1]:.2f}, Z={punto[2]:.2f}")



        

        


        

    
