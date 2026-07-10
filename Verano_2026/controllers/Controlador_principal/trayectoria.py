import numpy as np 
import math as mt 

class control_trayectoria:

    def __init__(self):

        self.fov=0.87
        ladox=4.5 #0.4
        ladoy=4.5
        ladoz=1.0
        self.altura_vuelo=ladoz/2
        self.perimetro=2*(ladox+ladoy)
        self.area=(ladox*ladoy)
        self.d= 2.5 #radio de trayectoria circular


        self.posx=1.0   #posicion del obj x
        self.posy=1.0   #posicion del obj y

        self.posx_ini=-2.0
        self.posy_ini=-2.0

    def trayectoria_inicial(self,numero_puntos,clase_trayectoria):
        if clase_trayectoria=="circular":
            trayectoria=self.puntos_trayectoria_circular(numero_puntos)
        elif clase_trayectoria=="cuadrada":
            trayectoria=self.trayectoria_cuadrada(numero_puntos)
        elif clase_trayectoria=="Random":
            trayectoria=self.trayectoria_random(numero_puntos)
        elif clase_trayectoria=="circulo_ruido":
            trayectoria=self.trayectoria_circulo_r(numero_puntos)
        elif clase_trayectoria=="cuadrado_ruido":
            trayectoria=self.trayectoria_cuadrado_r(numero_puntos)
        return trayectoria
    
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
        
        angulo_inicio = mt.atan2(self.posy_ini - self.posy, self.posx_ini - self.posx)

        for n in range(numero_puntos):
            angulo_actual=angulo_inicio + n*separacion
            xn=(self.d*mt.cos(angulo_actual))+self.posx
            yn=(self.d*mt.sin(angulo_actual))+self.posy
            zn=self.altura_vuelo
            trayectoria.append([xn,yn,zn])
        return trayectoria

    def trayectoria_cuadrada(self, numero_puntos):
        trayectoria = []
        l = 4.5
        aristas = [
            (self.posx_ini, self.posy_ini),
            (self.posx_ini + l, self.posy_ini),
            (self.posx_ini + l, self.posy_ini + l),
            (self.posx_ini, self.posy_ini + l)
        ]
        
        # Distancia entre puntos a lo largo del perímetro
        perimeter = 4 * l
        step = perimeter / numero_puntos
        
        for i in range(numero_puntos):
            d = i * step
            if d < l:
                x = aristas[0][0] + d
                y = aristas[0][1]
            elif d < 2 * l:
                x = aristas[1][0]
                y = aristas[1][1] + (d - l)
            elif d < 3 * l:
                x = aristas[2][0] - (d - 2 * l)
                y = aristas[2][1]
            else:
                x = aristas[3][0]
                y = aristas[3][1] - (d - 3 * l)
            trayectoria.append([x, y, self.altura_vuelo])
            
        return trayectoria
    
    def trayectoria_random(self,numero_puntos):
        posix=self.posx_ini
        posiy=self.posy_ini
        separacion=(self.perimetro)/numero_puntos
        trayectoria=[]
        #Posicion inicial
        dr=2.0
        trayectoria_ini=self.puntos_trayectoria_circular(numero_puntos)
        trayectoria=self.ruido(trayectoria_ini,dr)
        
        return trayectoria
    
    def trayectoria_circulo_r(self,numero_puntos):
        separacion=(2.0*mt.pi)/numero_puntos
        trayectoria=[]
        dr=0.5
        angulo_inicio = mt.atan2(self.posy_ini - self.posy, self.posx_ini - self.posx)

        for n in range(numero_puntos):
            angulo_actual=angulo_inicio + n*separacion
            xn=(self.d*mt.cos(angulo_actual))+self.posx
            yn=(self.d*mt.sin(angulo_actual))+self.posy
            zn=self.altura_vuelo
            trayectoria.append([xn,yn,zn])
        trayectoria=self.ruido(trayectoria,dr)
        
        return trayectoria
    
    def trayectoria_cuadrado_r(self, numero_puntos):
        trayectoria = self.trayectoria_cuadrada(numero_puntos)
        dr = 0.7
        trayectoria = self.ruido(trayectoria, dr)
        return trayectoria 
        
    def ruido (self,trayectoria,dr): #dr=cunato ruido se genera en los puntos 

        puntos_opt=[]
        
        
        for i in range(len(trayectoria)):
            x=trayectoria[i][0]
            y=trayectoria[i][1]
            z=trayectoria[i][2]
            
            if i%2==0:
                x += np.random.uniform(-dr, dr)
                y += np.random.uniform(-dr, dr)
            
            trayectoria[i]=[x,y,z]   
        return trayectoria

if __name__ == "__main__":
    trayectoria=control_trayectoria()
    numero_puntos=trayectoria.puntos_necesarios()
    # "circular"
    # "cuadrada"
    # "Random"
    # "circulo_ruido"
    # "cuadrado_ruido"

    tray=trayectoria.trayectoria_inicial(numero_puntos,"circular")
    for i, punto in enumerate(tray):
        print(f"Punto {i+1}: X={punto[0]:.2f}, Y={punto[1]:.2f}, Z={punto[2]:.2f}")



        

        


        

    
