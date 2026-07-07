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

# Rutas dinámicas
dir_controlador = os.path.dirname(os.path.abspath(__file__))
dir_actual = os.path.dirname(os.path.abspath(__file__))
ruta_historial = os.path.join(dir_actual, "historial_evaluaciones.csv")
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

# Ya no usamos angulos_base globales para forzar la trayectoria circular
# angulos_base = []

def fun_costo(p):
    global contador
    contador+=1
    print(f"EVALUACION {contador}")
    peso=3.2

    # 1. Reconstruir a partir del vector p (ahora contiene [x0, y0, x1, y1, ...])
    # Como son 2 variables (x, y) por punto
    wp_xy = np.reshape(p, (-1, 2)).tolist()

    wp = []
    h=0.0 #Penalizacion limites de volumen
    penalizacion_suavidad = 0.0
    
    ALTURA_FIJA = 0.55

    for i, xy in enumerate(wp_xy):
        x = xy[0]
        y = xy[1]
        z = ALTURA_FIJA
        
        # Calcular radio real actual para restricciones
        r = mt.sqrt((x - 1.0)**2 + (y - 1.0)**2)

        # Restricciones en R (aunque los limites del optimizador ya evitan esto, lo dejamos por seguridad)
        if r<1.2:
            h+=abs(1.2-r)*500.0
        elif r>5.5:
            h+=abs(5.5-r)*500.0

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
    minim=0.4
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
    with open(ruta_mision, "w") as f:
        json.dump(diccionario, f)
    
    # 3. Llamar a Webots (subprocess)
    detector.limpiar_fotos()
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
    
    ptj=peso*puntaje
    
    # El costo ahora se basa en el puntaje, volumen e irregularidades (ruidos)
    costo = -ptj+h+penalizacion_distancia+penalizacion_suavidad
    
    costos.append(costo)

    #tiempo de vuelo, puntaje 
    print(f"Tiempo de vuelo: {tiempo_total:.2f} , Distancia teorica: {distancia_teorica:.2f} , puntaje: {puntaje:.1f}, costo: {costo:.2f}  , |puntos obtenidos: {ptj:.2f}|")
  
    with open(ruta_historial, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([contador, costo] + np.ravel(wp_xy).tolist())

    x_vals = [w[0] for w in wp]
    y_vals = [w[1] for w in wp]
    z_vals=[w[2] for w in wp]

    x_vals.append(x_vals[0])
    y_vals.append(y_vals[0])
    z_vals.append(z_vals[0])
    
    plt.figure(figsize=(6,6))
    plt.plot(x_vals, y_vals, marker='x', color='blue', label='Ruta')
    for x, y, z in zip(x_vals[:-1], y_vals[:-1], z_vals[:-1]):
        plt.text(x, y, f"({x:.1f}, {y:.1f},{z:.1f})", fontsize=8, ha='left', va='bottom', color='black')

    plt.plot(1.0, 1.0, marker='*', color='gold', markersize=15, label='Objeto') 
    ax = plt.gca()
    ax.add_patch(plt.Circle((1.0, 1.0), 1.2, color='red', fill=False, linestyle='--'))
    ax.add_patch(plt.Circle((1.0, 1.0), 5.5, color='green', fill=False, linestyle='--'))

    plt.xlim(-5, 7)
    plt.ylim(-5, 7)
    plt.grid(True)
    plt.title(f"Iteración {contador} - Costo: {costo:.2f}")
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
    
    puntos_opt=[]
    
    # Rellenamos el vector inicial en modo Cartesiano (x, y)
    for i in range(len(puntos)):
        x=puntos[i][0]
        y=puntos[i][1]
        
        # Agregamos ruido aleatorio seguro para que la trayectoria inicie "extraña"
        if i%2==0:
            x += np.random.uniform(-0.3, 0.3)
            y += np.random.uniform(-0.3, 0.3)
            
        puntos_opt.append([x, y])

    puntos2d=np.array(puntos_opt)
    wp_in=np.ravel(puntos2d)
    
    print(f" Optimizando con COBYLA (2 variables (x,y)/pto, altura fija en {ALTURA_FIJA})... ")
    
    # Inicializar el archivo CSV para el historial
    with open(ruta_historial, mode='w', newline='') as file:
        writer = csv.writer(file)
        # Crear los encabezados: Evaluacion, Costo, x0, y0, x1, y1...
        encabezados = ["Evaluacion", "Costo"]
        for i in range(len(wp_in)//2):
            encabezados.extend([f"x{i}", f"y{i}"])
        writer.writerow(encabezados)

    # Restricciones para COBYLA: (x-1)^2 + (y-1)^2 >= 1.2^2  y  (x-1)^2 + (y-1)^2 <= 5.5^2
    restricciones = []
    # Generar funciones usando cierres (closures) seguros
    def crear_restriccion_min(idx):
        return lambda p: (p[idx] - 1.0)**2 + (p[idx+1] - 1.0)**2 - 1.2**2
    def crear_restriccion_max(idx):
        return lambda p: 5.5**2 - ((p[idx] - 1.0)**2 + (p[idx+1] - 1.0)**2)
        
    for j in range(0, len(wp_in), 2):
        restricciones.append({'type': 'ineq', 'fun': crear_restriccion_min(j)})
        restricciones.append({'type': 'ineq', 'fun': crear_restriccion_max(j)})

    # Ejecutar COBYLA
    resultado = minimize(fun_costo, x0=wp_in, method='COBYLA', constraints=restricciones, options={'maxiter': 50, 'disp': True})
    
    print("\n")
    print(f"RESULTADO FINAL (XY):\n{np.round(np.reshape(resultado.x, (-1, 2)), 2)}")
    
    # Graficar la mejor convergencia (costo mínimo hasta cada iteración)
    mejores_costos = np.minimum.accumulate(costos)
    plt.plot(range(1, len(mejores_costos) + 1), mejores_costos, marker='o', color='blue', label='Mejor Costo')
    plt.plot(range(1, len(costos) + 1), costos, color='lightgray', alpha=0.5, label='Costo Evaluado')
    plt.xlabel("Evaluaciones")
    plt.ylabel("Costo (Error)")
    plt.title("Convergencia del Optimizador")
    plt.legend()
    plt.grid(True)
    plt.savefig(ruta_convergencia)
    # plt.show() # Opcional: mostrar la gráfica al final

    ty_opt=np.reshape(resultado.x, (-1, 2)).tolist()

    # Recalculamos a Cartesianas para guardar la misión completa en 3D
    wp_final = []
    ALTURA_FIJA = 0.75
    for i, xy in enumerate(ty_opt):
        z = ALTURA_FIJA
        x = xy[0]
        y = xy[1]
        wp_final.append([x, y, z])

    x_opt = [w[0] for w in wp_final]
    y_opt = [w[1] for w in wp_final]
    x_opt.append(x_opt[0])
    y_opt.append(y_opt[0])

    plt.figure(figsize=(8,8))
    plt.plot(x_opt, y_opt, marker='o', color='magenta', linewidth=1, label='Ruta Óptima')
    plt.text(-5.0,5.0,f"Costo total: {resultado.fun:.2f} ",fontsize=12, ha='left', va='bottom', color='red')
    for x, y in zip(x_opt[:-1], y_opt[:-1]):
        plt.text(x, y, f"({x:.1f}, {y:.1f})", fontsize=6, ha='left', va='bottom', color='black')
    plt.plot(1.0, 1.0, marker='*', color='gold', markersize=15, label='Objeto') 
    ax = plt.gca()
    ax.add_patch(plt.Circle((1.0, 1.0), 1.5, color='red', fill=False, linestyle='--'))
    ax.add_patch(plt.Circle((1.0, 1.0), 5.5, color='green', fill=False, linestyle='--'))

    plt.xlim(-5, 7)
    plt.ylim(-5, 7)
    plt.grid(True)
    plt.legend()
    plt.title("Trayectoria Final Optimizada")
    ruta_trayectoria_final = os.path.join(ruta_base, "trayectoria_final.png")
    plt.savefig(ruta_trayectoria_final)
    # plt.show() # Opcional: mostrar
    
    diccionario={
        "waypoints":wp_final
    }
    with open(ruta_mision, "w") as f:
        json.dump(diccionario, f)
    
    # 3. Llamar a Webots (subprocess)
    video_output= os.path.join(ruta_historia, f"video_final_{contador}.mp4")
    comando=[webots_path,  ruta_mundo]
    print("SIMULACION FINAL")
    subprocess.run(comando)
