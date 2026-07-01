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

# Rutas dinámicas
dir_controlador = os.path.dirname(os.path.abspath(__file__))
ruta_base = os.path.abspath(os.path.join(dir_controlador, "..", ".."))

# Determinar ruta de Webots
webots_path = "webots"
posibles_rutas = [
    r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe",
    r"C:\Program Files\Webots\bin\webots.exe",
    r"C:\Program Files\Webots\msys64\mingw64\bin\webotsw.exe",
    r"C:\Program Files\Webots\bin\webotsw.exe"
]
for r in posibles_rutas:
    if os.path.exists(r):
        webots_path = r
        break

ruta_mision = os.path.join(ruta_base, "mision.json")
ruta_reporte = os.path.join(ruta_base, "reporte_vuelo.json")
ruta_mundo = os.path.join(ruta_base, "worlds", "crazyflie.wbt")
ruta_historia = os.path.join(ruta_base, "Historia_rutas")
ruta_convergencia = os.path.join(ruta_base, "convergencia.png")

detector=YoloDetector()
tray_obj = control_trayectoria()

contador=0
costos=[]
def fun_costo(p):
    global contador 
    contador+=1
    print(f"ITERACION {contador}")
    # 1. Reconstruir a 4D a partir de 3D (x, y, z)
    wp3d = np.reshape(p, (-1, 3)).tolist()
    wp = []
    for w in wp3d:
        x = w[0]
        y = w[1]
        z = w[2]
        dx = 1.0 - x
        dy = 1.0 - y
        yaw = mt.atan2(dy, dx)
        wp.append([x, y, z, yaw])
    
    # 2. Escribir 'mision.json'
    diccionario={
        "waypoints":wp
    }
    with open(ruta_mision, "w") as f:
        json.dump(diccionario, f)
    
    # 3. Llamar a Webots (subprocess)
    comando=[webots_path, "--mode=fast","--no-rendering","--minimize", "--batch", ruta_mundo]
    print("SIMULACION")
    subprocess.run(comando)
    
    # 4. Leer 'reporte_vuelo.json'
    with open(ruta_reporte, "r") as f:
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
        if d<1.2:
            co += 1000.0 * (1.2 - d)**2
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
    ax.add_patch(plt.Circle((1.0, 1.0), 1.2, color='red', fill=False, linestyle='--'))
    ax.add_patch(plt.Circle((1.0, 1.0), 5.5, color='green', fill=False, linestyle='--'))

    plt.xlim(-5, 7)
    plt.ylim(-5, 7)
    plt.grid(True)
    plt.title(f"Iteración {contador} - Costo: {round(costo, 2)}")
    plt.savefig(os.path.join(ruta_historia, f"iteracion_{contador}.png"))
    plt.close()

    return costo

if __name__ == "__main__":

    if os.path.exists(ruta_historia) is False:
        os.makedirs(ruta_historia)
    else:
        shutil.rmtree(ruta_historia)
        os.makedirs(ruta_historia)
        
    num_puntos=tray_obj.puntos_necesarios()
    puntos=tray_obj.puntos_trayectoria_circular(num_puntos)
    
    puntos3d=[]
   
    for i in range(len(puntos)):
        x=puntos[i][0]
        y=puntos[i][1]
        z=puntos[i][2]
        puntos3d.append([x, y, z])

    puntos3d=np.array(puntos3d)
    wp_in=np.ravel(puntos3d)
    

    N = len(wp_in)
    simplex_inicial = np.zeros((N + 1, N))
    simplex_inicial[0] = wp_in
    radio_busqueda = 1.5 # 1.5 metros de salto inicial para escapar del ruido de YOLO
    for i in range(N):
        simplex_inicial[i+1] = np.copy(wp_in)
        simplex_inicial[i+1][i] += radio_busqueda
    # -----------------------------------------------
    
    print(f" optimizando con {len(puntos3d)} puntos de control (en 3D) ")
    # Usamos Nelder-Mead con un simplex inicial grande. 
    # fatol y xatol aumentados para que no pierda tiempo en precisiones milimétricas que YOLO no puede percibir.
    resultado=minimize(fun_costo, wp_in, method="Nelder-Mead", options={'maxiter':1000, 'disp':True, 'initial_simplex': simplex_inicial, 'adaptive': True, 'xatol': 0.1, 'fatol': 0.1})
    print("\n")
    print(f"RESULTADO:{np.reshape(resultado.x,(-1,3))}")
    

    plt.plot(range(1, len(costos) + 1), costos, marker='o')
    plt.xlabel("Iteraciones")
    plt.ylabel("Costo (Error)")
    plt.title("Convergencia del Algoritmo de Optimización")
    plt.grid(True)
    plt.savefig(ruta_convergencia)
    plt.show()


    