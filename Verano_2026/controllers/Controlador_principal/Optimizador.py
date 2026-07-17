from scipy.optimize import _minimize
import numpy as np 
import json 
import subprocess 
import math as mt
from scipy.optimize import minimize,differential_evolution
from trayectoria import control_trayectoria
import sys
from yolo_detector import YoloDetector
import matplotlib.pyplot as plt
import shutil
import os 
from scipy.optimize import differential_evolution
import csv
from graficas import Graficar_trayectoria,graficas_finales


                                #--------------------- Rutas archivos ---------------------#
dir_controlador = os.path.dirname(os.path.abspath(__file__))
dir_actual = os.path.dirname(os.path.abspath(__file__))
ruta_historial = os.path.join(dir_actual, "historial_evaluaciones.csv")
ruta_base = os.path.abspath(os.path.join(dir_controlador, "..", ".."))
ruta_resultados=os.path.join(ruta_base, "Resultados")
if not os.path.exists(ruta_resultados):
    os.makedirs(ruta_resultados)

ruta_historia = os.path.join(ruta_resultados, "Historial_rutas")
ruta_historial = os.path.join(ruta_resultados, "historial_evaluaciones.csv")

if os.path.exists(ruta_historia) is False:
    os.makedirs(ruta_historia, mode=0o777)
else:
    try:
        shutil.rmtree(ruta_historia, ignore_errors=True)
    except OSError as e:
        print(f"Error al borrar la carpeta: {e}")
        # Opcional: intentar con chmod si el error es por permisos
        os.chmod(ruta_historia, 0o777)
        try:
            shutil.rmtree(ruta_historia)
        except OSError as e2:
            print(f"Error al borrar de nuevo: {e2}")
    os.makedirs(ruta_historia, mode=0o777, exist_ok=True)




webots_path = "webots"
path = [
    "/usr/local/webots/msys64/mingw64/bin/webots.exe",
    "/usr/local/webots/bin/webots.exe",
    "/usr/local/webots/msys64/mingw64/bin/webotsw.exe",
    "/usr/local/webots/bin/webotsw.exe",
    "C:/Program Files/Webots/msys64/mingw64/bin/webots.exe",
    "C:/Program Files/Webots/msys64/mingw64/bin/webotsw.exe",
    "C:/Program Files/Webots/bin/webots.exe",
    "C:/Program Files/Webots/bin/webotsw.exe"
]
for r in path:
    if os.path.exists(r):
        webots_path = r
        break

rt_mision = os.path.join(ruta_base, "mision.json")
rt_reporte = os.path.join(ruta_base, "reporte_vuelo.json")
rt_mundo = os.path.join(ruta_base, "worlds", "crazyflie.wbt")

ruta_convergencia = os.path.join(ruta_resultados, "convergencia.png")
                                #--------------------- Inicialización ---------------------#
detector=YoloDetector()
tray_obj = control_trayectoria()
contador=0
costos=[]
altura=0.47
                                #--------------------- Funcion costo ---------------------#
def fun_costo(waypointsrt):
    global contador
    global altura
    contador+=1
    print(f"Evaluacion no: {contador}")
    peso=3.2
    radio_min=1.8
    radio_max=4.5

  
    wp_xy = np.reshape(waypointsrt, (-1, 2)).tolist()

    wp = []
    wp_guardar = []
    h=0.0 #Penalizacion limites de volumen
    penalizacion_dmax = 0.0

    for i, xy in enumerate(wp_xy):
        x = xy[0]
        y = xy[1]
        z = altura
        
        # Calcular radio real actual para restricciones
        r = mt.sqrt((x - 1.0)**2 + (y - 1.0)**2)

        # Restricciones en r
        if r<radio_min:
            h+=abs(radio_min-r)*500.0
        elif r>radio_max:
            h+=abs(radio_max-r)*500.0

        # Penalización por cambios bruscos 
        if i > 0:
            x_prev = wp_xy[i-1][0]
            y_prev = wp_xy[i-1][1]
            dist_prev = mt.sqrt((x - x_prev)**2 + (y - y_prev)**2)
            # Penalizar si la distancia entre waypoints es muy grande 
            if dist_prev > 2.5:
                penalizacion_dmax += (dist_prev - 2.5) * 100.0
            
        yaw = mt.atan2(1.0 - y, 1.0 - x)
        wp.append([x, y, z])
        wp_guardar.extend([x, y, z, yaw])

    # Calcular distancia teórica de la ruta (sin ruido de simulación)
    distancia_teorica = 0.0
    segmentos=[]
    for i in range(len(wp)-1):
        distancia_teorica += mt.sqrt((wp[i+1][0]-wp[i][0])**2 + (wp[i+1][1]-wp[i][1])**2)
        segmentos.append(mt.sqrt((wp[i+1][0]-wp[i][0])**2 + (wp[i+1][1]-wp[i][1])**2))
    # Sumar el tramo final al inicio para cerrar el circuito
    distancia_teorica += mt.sqrt((wp[0][0]-wp[-1][0])**2 + (wp[0][1]-wp[-1][1])**2)
    segmentos.append(mt.sqrt((wp[0][0]-wp[-1][0])**2 + (wp[0][1]-wp[-1][1])**2))
    
    penalizacion_distancia=0.0
    minim=0.5
    for i in range(len(wp)):
        for j in range(i+1,len(wp)):
            di=mt.sqrt((wp[i][0]-wp[j][0])**2+(wp[i][1]-wp[j][1])**2)
            if di<minim:
                penalizacion_distancia+=(minim-di)*500.0
            
        
    wp_con_inicio = [[-2.0, -2.0, 0.85]] + wp + [wp[0]]
    for i in range(len(wp_con_inicio) - 1):
        p1 = wp_con_inicio[i]
        p2 = wp_con_inicio[i+1]
        x1, y1 = p1[0], p1[1]
        x2, y2 = p2[0], p2[1]
        cx, cy = 1.0, 1.0
        dx = x2 - x1
        dy = y2 - y1
        l2 = dx*dx + dy*dy
        if l2 > 0:
            t = max(0, min(1, ((cx - x1)*dx + (cy - y1)*dy) / l2))
            px = x1 + t * dx
            py = y1 + t * dy
            dist_seg = mt.sqrt((px - cx)**2 + (py - cy)**2)
            if dist_seg < 1.75:
                h += abs(1.75 - dist_seg) * 500.0

    if h>0:
      ptj=peso*(-100.0)
      costo=-ptj+h+penalizacion_distancia+penalizacion_dmax
      costos.append(costo)
      print(f"Posicion invalida: {contador} - Costo: {costo:.2f}")
      
      with open(ruta_historial, mode='a', newline='') as file:
          writer = csv.writer(file)
          writer.writerow([contador, costo, 0.0, h, penalizacion_distancia, penalizacion_dmax, distancia_teorica, 0.0] + wp_guardar)
      return costo 
    
    # 2. Escribir 'mision.json'
    diccionario={
        "waypoints":wp
    }
    with open(rt_mision, "w") as f:
        json.dump(diccionario, f)
    
    # 3. Llamar a Webots (subprocess)
    detector.limpiar_fotos()
    comando=[webots_path, "--mode=fast","--no-rendering","--minimize", "--batch", rt_mundo]
    print("SIMULACION")
    subprocess.run(comando)
    
    # 4. Leer 'reporte_vuelo.json'
    with open(rt_reporte, "r") as f:
        diccionario2=json.load(f)
        t_vuelo=diccionario2["t_vuelo"]
        pos_inicial=diccionario2["pos_inicial"]
        pos_final=diccionario2["pos_final"]
        puntos=diccionario2["puntos"]
        tiempo_total=diccionario2["tiempo_total"]

    # 5. Retornar el costo
    puntaje=detector.imagenes_capturadas()
    
    # PESOS para gradiente continuo
    peso_fotos = 3.2
    peso_dist = 0.5
    peso_tiempo = 0.2
    
    ptj = peso_fotos * puntaje
    
   
    costo = -ptj + h + penalizacion_distancia + penalizacion_dmax + (peso_dist * distancia_teorica) + (peso_tiempo * tiempo_total)
    
    costos.append(costo)

    #tiempo de vuelo, puntaje 
    print(f"Tiempo de vuelo: {tiempo_total:.2f} , Distancia teorica: {distancia_teorica:.2f} , puntaje: {puntaje:.1f}, costo: {costo:.2f} ")
  
    with open(ruta_historial, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([contador, costo, puntaje, h, penalizacion_distancia, penalizacion_dmax, distancia_teorica, tiempo_total] + wp_guardar)

    Graficar_trayectoria(puntos,contador,costo,ruta_historia,radio_min,radio_max)


    return costo

if __name__ == "__main__":
  
    altura=0.45
    rmin=1.8
    rmax=4.5
        
    num_puntos=tray_obj.puntos_necesarios()
    #elegir trayectoria
    # "circular"
    # "cuadrada"
    # "Random"
    # "circulo_ruido"
    # "cuadrado_ruido"

    puntos3d=tray_obj.trayectoria_inicial(num_puntos,"circular")
    puntos2d=[[p[0],p[1]] for p in puntos3d]

    wp_in=np.ravel(puntos2d)
    print(f" Optimizando con COBYLA  ")
    
    # Inicializar el archivo CSV 
    with open(ruta_historial, mode='w', newline='') as file:
        writer = csv.writer(file)
        # Crear los encabezados
        encabezados = ["Evaluacion", "Costo", "Puntaje_vision", "restriccion_area", "Penalizacion_dmin", "Penalizacion_dmax", "Distancia_vuelo", "Tiempo_Vuelo"]
        for i in range(len(wp_in)//2):
            encabezados.extend([f"x{i}", f"y{i}", f"z{i}", f"yaw{i}"])
        writer.writerow(encabezados)

    # Restricciones para COBYLA: (x-1)^2 + (y-1)^2 >= 1.8^2  y  (x-1)^2 + (y-1)^2 <= 5.5^2
    restricciones = []
    
    #Funciones para restriccion cobyla
    def crear_restriccion_min(idx):
        return lambda p: (p[idx] - 1.0)**2 + (p[idx+1] - 1.0)**2 - 1.8**2
    def crear_restriccion_max(idx):
        return lambda p: 5.5**2 - ((p[idx] - 1.0)**2 + (p[idx+1] - 1.0)**2)
        
    for j in range(0, len(wp_in), 2):
        restricciones.append({'type': 'ineq', 'fun': crear_restriccion_min(j)})
        restricciones.append({'type': 'ineq', 'fun': crear_restriccion_max(j)})

    # Símplex Nelder-Mead
    N = len(wp_in)
    simplex = np.zeros((N + 1, N))
    simplex[0] = wp_in
    for i in range(N):
        p = np.copy(wp_in)
        p[i] += 1.0 
        simplex[i + 1] = p

    #Limites differential evolution
    lmin=-4.5
    lmax=6.5
    lims=[(lmin,lmax) for _ in range(len(wp_in))]
    
    # Ejecutar differential evolution
    #resultado = differential_evolution(fun_costo, lims, strategy='best1bin', maxiter=15, popsize=3, tol=0.01, mutation=(0.5, 1), recombination=0.7, seed=None, disp=True, polish=True, workers=1, updating='immediate')
    

    # Ejecutar SLSQP
    resultado = minimize(fun_costo, x0=wp_in, method='SLSQP', bounds=lims, constraints=restricciones, options={'maxiter': 100, 'disp': True,'eps':0.2})
    
    print(f"\n--- COSTO FINAL: {resultado.fun:.2f} ---")
    
    ty_opt=np.reshape(resultado.x, (-1, 2)).tolist()
    wp_final = []

    print("RESULTADO FINAL (X, Y, Z, YAW):")
    wp_guardar_final = []
    for i, xy in enumerate(ty_opt):
        z = altura
        x = xy[0]
        y = xy[1]
        yaw = mt.atan2(1.0 - y, 1.0 - x)
        print(f"P{i}: X={x:.2f}, Y={y:.2f}, Z={z:.2f}, Yaw={yaw:.2f}")
        wp_final.append([x, y, z])
        wp_guardar_final.extend([x, y, z, yaw])
        
    with open(ruta_historial, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["OPTIMO", resultado.fun, "", "", "", "", "", ""] + wp_guardar_final)

    graficas_finales(costos,resultado,ruta_convergencia,ruta_base,altura,rmin,rmax)

    
    diccionario={
        "waypoints":wp_final
    }
    with open(rt_mision, "w") as f:
        json.dump(diccionario, f)
    
    # 3. Llamar a Webots (subprocess)
    video_output= os.path.join(ruta_historia, f"video_final_{contador}.mp4")
    comando=[webots_path,  rt_mundo]
    print("SIMULACION FINAL")
    subprocess.run(comando)

