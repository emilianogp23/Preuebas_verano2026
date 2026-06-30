from scipy.optimize import _minimize
import numpy as np 
import json 
import subprocess 
import math as mt
from scipy.optimize import minimize
from trayectoria import control_trayectoria
import sys
from yolo_detector import YoloDetector
import matplotlib.pyplot as plt
import shutil
import os 


detector=YoloDetector()

contador=0
costos=[]
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
    puntaje=detector.imagenes_capturadas()
    peso=2.0
    ptj=peso*puntaje
    
    # Calcular distancia teórica de la ruta (sin ruido de simulación)
    distancia_teorica = 0.0
    for i in range(len(wp)-1):
        distancia_teorica += mt.sqrt((wp[i+1][0]-wp[i][0])**2 + (wp[i+1][1]-wp[i][1])**2)
    # Sumar el tramo final al inicio para cerrar el circuito
    distancia_teorica += mt.sqrt((wp[0][0]-wp[-1][0])**2 + (wp[0][1]-wp[-1][1])**2)
    
    # El costo ahora se basa en la distancia teórica (suave) en lugar del tiempo simulado (ruidoso)
    costo = distancia_teorica - ptj
    co=0.0
    for w in wp:
        w1=w[0]
        w2=w[1]
        d=mt.sqrt((w1-1.0)**2+(w2-1.0)**2)
        if d<2.0:
            co += 1000.0 * (2.0 - d)**2
        elif d>5.5:
            co += 1000.0 * (d - 5.5)**2
    costo=costo+co
    costos.append(costo)

    #tiempo de vuelo, puntaje 
    print(f"Tiempo de vuelo: {tiempo_total} , Distancia teorica: {distancia_teorica} , puntaje: {puntaje}, costo: {costo} , |costo penalizacion: {co} , |puntos obtenidos: {ptj}|")
  

    x_vals = [w[0] for w in wp]
    y_vals = [w[1] for w in wp]
    x_vals.append(x_vals[0])
    y_vals.append(y_vals[0])

    plt.figure(figsize=(6,6))
    plt.plot(x_vals, y_vals, marker='x', color='blue', label='Ruta')
    plt.plot(1.0, 1.0, marker='*', color='gold', markersize=15, label='Objeto') 
    ax = plt.gca()
    ax.add_patch(plt.Circle((1.0, 1.0), 2.0, color='red', fill=False, linestyle='--'))
    ax.add_patch(plt.Circle((1.0, 1.0), 5.5, color='green', fill=False, linestyle='--'))

    plt.xlim(-5, 7)
    plt.ylim(-5, 7)
    plt.grid(True)
    plt.title(f"Iteración {contador} - Costo: {round(costo, 2)}")
    plt.savefig(f"Historia_rutas/iteracion_{contador}.png")
    plt.close()

    return costo

if __name__ == "__main__":

    if os.path.exists("Historia_rutas") is False:
        os.makedirs("Historia_rutas")
    else:
        shutil.rmtree("Historia_rutas")
        os.makedirs("Historia_rutas")
        
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
    

    N = len(wp_in)
    simplex_inicial = np.zeros((N + 1, N))
    simplex_inicial[0] = wp_in
    radio_busqueda = 0.5 # 50 centímetros (o 0.5 radianes) de salto inicial
    for i in range(N):
        simplex_inicial[i+1] = np.copy(wp_in)
        simplex_inicial[i+1][i] += radio_busqueda
    # -----------------------------------------------
    
    print(f" optimizando con {len(puntos4d)} puntos de control ")
    resultado=minimize(fun_costo, wp_in,method="Nelder-Mead",options={'maxiter':1000,'disp':True, 'initial_simplex': simplex_inicial})
    print("\n")
    print(f"RESULTADO:{np.reshape(resultado.x,(-1,4))}")
    

    plt.plot(range(1, len(costos) + 1), costos, marker='o')
    plt.xlabel("Iteraciones")
    plt.ylabel("Costo (Error)")
    plt.title("Convergencia del Algoritmo de Optimización")
    plt.grid(True)
    plt.savefig("convergencia.png")
    plt.show()


    