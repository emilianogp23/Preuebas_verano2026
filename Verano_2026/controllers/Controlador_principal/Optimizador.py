from scipy.optimize import _minimize
import numpy as np 
import json 
import subprocess 
import math as mt
from scipy.optimize import minimize
from trayectoria import control_trayectoria
import sys
contador=0
def fun_costo(p):
    global contador 
    contador+=1
    print(f"ITERACION {contador}")
# 1. Reconstruir a 4D
    wp = np.reshape(p, (-1, 4)).tolist()
    
    # 2. Escribir 'mision.json'
    diccionario={
        "waypoints":wp
    }
    with open("mision.json", "w") as f:
        json.dump(diccionario, f)
    
    # 3. Llamar a Webots (subprocess)
    comando=["webots", "--mode=fast","--no-rendering","--minimize", "--batch", "/home/jpirmz/Documents/PR_Bebop/Pruebas_verano2026/Verano_2026/worlds/crazyflie.wbt"]
    print("SIMULACION")
    subprocess.run(comando)
    
    # 4. Leer 'reporte_vuelo.json'
    with open("reporte_vuelo.json", "r") as f:
        diccionario2=json.load(f)
        t_vuelo=diccionario2["t_vuelo"]
        pos_inicial=diccionario2["pos_inicial"]
        pos_final=diccionario2["pos_final"]
        puntos=diccionario2["puntos"]
        tiempo_total=diccionario2["tiempo_total"]

    # 5. Retornar el costo
    return t_vuelo

if __name__ == "__main__":
    tray_obj=control_trayectoria()
    num_puntos=tray_obj.puntos_necesarios()
    puntos=tray_obj.puntos_trayectoria_circular(num_puntos)
    waypoints=tray_obj.puntos_trayectoria_circular(num_puntos)
    puntos4d=[]
   
    for i in range(len(puntos)):
        x=puntos[i][0]
        y=puntos[i][1]
        z=puntos[i][2]
        dy=1.0-y
        dx=1.0-x
        yaw=mt.atan2(dy,dx)
        puntos4d.append([x,y,z,yaw])

    puntos4d=np.array(puntos4d)
    wp_in=np.ravel(puntos4d)
    print(f" optimizando con {len(puntos4d)} puntos de control ")
    resultado=minimize(fun_costo, wp_in,method="Nelder-Mead",options={'maxiter':1,'disp':True})
    print("\n")
    print(f"RESULTADO:{np.reshape(resultado.x,(-1,4))}")
    

    

    