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
    "/usr/local/webots/bin/webotsw.exe"
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
    radio_min=1.2
    radio_max=4.5

  
    wp_xy = np.reshape(waypointsrt, (-1, 2)).tolist()

    wp = []
    h=0.0 #Penalizacion limites de volumen
    penalizacion_suavidad = 0.0

    for i, xy in enumerate(wp_xy):
        x = xy[0]
        y = xy[1]
        z = altura
        
        # Calcular radio real actual para restricciones
        r = mt.sqrt((x - 1.0)**2 + (y - 1.0)**2)

        # Restricciones en R (aunque los limites del optimizador ya evitan esto, lo dejamos por seguridad)
        if r<radio_min:
            h+=abs(radio_min-r)*500.0
        elif r>radio_max:
            h+=abs(radio_max-r)*500.0

        # Penalización por cambios bruscos (Suavidad)
        if i > 0:
            x_prev = wp_xy[i-1][0]
            y_prev = wp_xy[i-1][1]
            dist_prev = mt.sqrt((x - x_prev)**2 + (y - y_prev)**2)
            # Penalizar si la distancia entre waypoints es muy grande (salto brusco)
            if dist_prev > 2.5:
                penalizacion_suavidad += (dist_prev - 2.5) * 100.0
            
        wp.append([x, y, z])

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
            
        
    # Penalizar si los segmentos entre waypoints cruzan la zona prohibida (radio 1.2)
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
            if dist_seg < 1.15:
                h += abs(1.15 - dist_seg) * 500.0

    if h>0:
      ptj=peso*(-100.0)
      costo=-ptj+h+penalizacion_distancia+penalizacion_suavidad
      costos.append(costo)
      print(f"Posicion invalida: {contador} - Costo: {costo:.2f}")
      
      with open(ruta_historial, mode='a', newline='') as file:
          writer = csv.writer(file)
          writer.writerow([contador, costo] + np.ravel(wp_xy).tolist())
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
    
    # El costo ahora incluye la distancia y el tiempo para que COBYLA tenga un gradiente que seguir
    costo = -ptj + h + penalizacion_distancia + penalizacion_suavidad + (peso_dist * distancia_teorica) + (peso_tiempo * tiempo_total)
    
    costos.append(costo)

    #tiempo de vuelo, puntaje 
    print(f"Tiempo de vuelo: {tiempo_total:.2f} , Distancia teorica: {distancia_teorica:.2f} , puntaje: {puntaje:.1f}, costo: {costo:.2f} ")
  
    with open(ruta_historial, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([contador, costo] + np.ravel(wp_xy).tolist())

    Graficar_trayectoria(puntos,contador,costo,ruta_historia,radio_min,radio_max)


    return costo

if __name__ == "__main__":
  
    altura=0.45
    rmin=1.2
    rmax=4.5
        
    num_puntos=tray_obj.puntos_necesarios()
    #elegir trayectoria
    # "circular"
    # "cuadrada"
    # "Random"
    # "circulo_ruido"
    # "cuadrado_ruido"

    puntos3d=tray_obj.trayectoria_inicial(num_puntos,"cuadrada")
    puntos2d=[[p[0],p[1]] for p in puntos3d]

    wp_in=np.ravel(puntos2d)
    print(f" Optimizando con COBYLA  ")
    
    # Inicializar el archivo CSV 
    with open(ruta_historial, mode='w', newline='') as file:
        writer = csv.writer(file)
        # Crear los encabezados
        encabezados = ["Evaluacion", "Costo"]
        for i in range(len(wp_in)//2):
            encabezados.extend([f"x{i}", f"y{i}"])
        writer.writerow(encabezados)

    # Restricciones para COBYLA: (x-1)^2 + (y-1)^2 >= 1.2^2  y  (x-1)^2 + (y-1)^2 <= 5.5^2
    restricciones = []
    
    #Funciones para restriccion cobyla
    def crear_restriccion_min(idx):
        return lambda p: (p[idx] - 1.0)**2 + (p[idx+1] - 1.0)**2 - 1.2**2
    def crear_restriccion_max(idx):
        return lambda p: 5.5**2 - ((p[idx] - 1.0)**2 + (p[idx+1] - 1.0)**2)
        
    for j in range(0, len(wp_in), 2):
        restricciones.append({'type': 'ineq', 'fun': crear_restriccion_min(j)})
        restricciones.append({'type': 'ineq', 'fun': crear_restriccion_max(j)})

    # Ejecutar COBYLA
    resultado = minimize(fun_costo, x0=wp_in, method='COBYLA', constraints=restricciones, options={'maxiter': 250, 'disp': True})
    
    print("\n")
    print(f"RESULTADO FINAL (X,Y,Z):\n{np.round(np.reshape(resultado.x, (-1, 3)), 2)}")


    ty_opt=np.reshape(resultado.x, (-1, 2)).tolist()
    wp_final = []

    for i, xy in enumerate(ty_opt):
        z = altura
        x = xy[0]
        y = xy[1]
        wp_final.append([x, y, z])

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

