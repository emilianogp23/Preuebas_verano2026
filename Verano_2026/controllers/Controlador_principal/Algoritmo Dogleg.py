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
import csv
from graficas import Graficar_trayectoria,graficas_finales

                                #--------------------- Rutas archivos ---------------------#
dir_controlador = os.path.dirname(os.path.abspath(__file__))
dir_actual = os.path.dirname(os.path.abspath(__file__))
ruta_base = os.path.abspath(os.path.join(dir_controlador, "..", ".."))
ruta_resultados = os.path.join(ruta_base, "Resultados")
if not os.path.exists(ruta_resultados):
    os.makedirs(ruta_resultados)

ruta_historia = os.path.join(ruta_resultados, "Historial_rutas")
ruta_historial = os.path.join(ruta_resultados, "historial_evaluaciones.csv")
ruta_log = os.path.join(ruta_resultados, "dogleg_log.txt")
ruta_convergencia = os.path.join(ruta_resultados, "convergencia.png")

if os.path.exists(ruta_historia) is False:
    os.makedirs(ruta_historia, mode=0o777)
else:
    try:
        shutil.rmtree(ruta_historia, ignore_errors=True)
    except OSError as e:
        print(f"Error al borrar la carpeta: {e}")
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

                                #--------------------- Inicialización ---------------------#
detector = YoloDetector()
tray_obj = control_trayectoria()
contador = 0
costos = []
altura = 0.47
fitness_historial = []

                                #--------------------- Funcion costo ---------------------#
def fun_costo(waypointsrt):
    global contador
    global altura
    contador += 1
    print(f"Evaluacion no: {contador}")
    peso = 3.2
    radio_min = 1.8
    radio_max = 4.5
  
    wp_xy = np.reshape(waypointsrt, (-1, 2)).tolist()

    wp = []
    wp_guardar = []
    h = 0.0 # Penalizacion limites de volumen
    penalizacion_dmax = 0.0

    for i, xy in enumerate(wp_xy):
        x = xy[0]
        y = xy[1]
        z = altura
        
        # Calcular radio real actual para restricciones
        r = mt.sqrt((x - 1.0)**2 + (y - 1.0)**2)

        # Restricciones en r
        if r < radio_min:
            h += abs(radio_min - r) * 500.0
        elif r > radio_max:
            h += abs(radio_max - r) * 500.0

        # Penalización por cambios bruscos 
        if i > 0:
            x_prev = wp_xy[i-1][0]
            y_prev = wp_xy[i-1][1]
            dist_prev = mt.sqrt((x - x_prev)**2 + (y - y_prev)**2)
            if dist_prev > 2.5:
                penalizacion_dmax += (dist_prev - 2.5) * 100.0
            
        yaw = mt.atan2(1.0 - y, 1.0 - x)
        wp.append([x, y, z])
        wp_guardar.extend([x, y, z, yaw])

    distancia_teorica = 0.0
    segmentos = []
    for i in range(len(wp)-1):
        distancia_teorica += mt.sqrt((wp[i+1][0]-wp[i][0])**2 + (wp[i+1][1]-wp[i][1])**2)
        segmentos.append(mt.sqrt((wp[i+1][0]-wp[i][0])**2 + (wp[i+1][1]-wp[i][1])**2))
    distancia_teorica += mt.sqrt((wp[0][0]-wp[-1][0])**2 + (wp[0][1]-wp[-1][1])**2)
    segmentos.append(mt.sqrt((wp[0][0]-wp[-1][0])**2 + (wp[0][1]-wp[-1][1])**2))
    
    penalizacion_distancia = 0.0
    minim = 0.5
    for i in range(len(wp)):
        for j in range(i+1, len(wp)):
            di = mt.sqrt((wp[i][0]-wp[j][0])**2 + (wp[i][1]-wp[j][1])**2)
            if di < minim:
                penalizacion_distancia += (minim - di) * 500.0
            
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

    if h > 0:
        ptj = peso * (-100.0)
        costo = -ptj + h + penalizacion_distancia + penalizacion_dmax
        costos.append(costo)
        print(f"Posicion invalida: {contador} - Costo: {costo:.2f}")
        
        with open(ruta_historial, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([contador, costo, 0.0, h, penalizacion_distancia, penalizacion_dmax, distancia_teorica, 0.0] + wp_guardar)
        return costo 
    
    diccionario = {"waypoints": wp}
    with open(rt_mision, "w") as f:
        json.dump(diccionario, f)
    
    detector.limpiar_fotos()
    comando = [webots_path, "--mode=fast", "--no-rendering", "--minimize", "--batch", rt_mundo]
    print("SIMULACION")
    subprocess.run(comando)
    
    with open(rt_reporte, "r") as f:
        diccionario2 = json.load(f)
        tiempo_total = diccionario2["tiempo_total"]
        puntos = diccionario2["puntos"]

    puntaje = detector.imagenes_capturadas()
    
    peso_fotos = 3.2
    peso_dist = 0.5
    peso_tiempo = 0.2
    ptj = peso_fotos * puntaje
    
    costo = -ptj + h + penalizacion_distancia + penalizacion_dmax + (peso_dist * distancia_teorica) + (peso_tiempo * tiempo_total)
    costos.append(costo)

    print(f"Tiempo de vuelo: {tiempo_total:.2f} , Distancia teorica: {distancia_teorica:.2f} , puntaje: {puntaje:.1f}, costo: {costo:.2f} ")
  
    with open(ruta_historial, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([contador, costo, puntaje, h, penalizacion_distancia, penalizacion_dmax, distancia_teorica, tiempo_total] + wp_guardar)

    Graficar_trayectoria(puntos, contador, costo, ruta_historia, radio_min, radio_max)
    return costo

# Funciones de Dogleg (gradiente, hessiana, pasos)
def gradiente(fun, x, h=0.05):
    print("Calculando gradiente")
    x = np.asarray(x, dtype=float)
    g = np.zeros_like(x)
    for i in range(len(x)):
        print(f"Variable {i+1}/{len(x)}")
        xp = np.copy(x)
        xm = np.copy(x)
        xp[i] += h
        xm[i] -= h
        fp = fun(xp)
        fm = fun(xm)
        g[i] = (fp - fm)/(2*h)
    return g

def hessiana_inicial(n):
    return np.eye(n)

def actualizar_hessiana_bfgs(H, s, y):
    s = s.reshape(-1,1)
    y = y.reshape(-1,1)
    ys = float(y.T @ s)
    if ys <= 1e-10:
        return H
    rho = 1.0 / ys
    I = np.eye(H.shape[0])
    H = (I - rho*s@y.T) @ H @ (I - rho*y@s.T) + rho*(s@s.T)
    return H

def paso_cauchy(g, H):
    numerador = np.dot(g, g)
    denominador = np.dot(g, H @ g)
    if denominador <= 1e-12:
        return np.zeros_like(g)
    alpha = numerador / denominador
    p = -alpha * g
    return p

def paso_newton(H, g):
    try:
        p = -np.linalg.solve(H, g)
    except np.linalg.LinAlgError:
        print("Hessiana singular. Se usa pseudo-inversa.")
        p = -np.linalg.pinv(H) @ g
    return p

def dogleg(g, H, Delta):
    p_u = paso_cauchy(g, H)
    p_b = paso_newton(H, g)
    if np.linalg.norm(p_b) <= Delta:
        print("Dogleg: usando paso de Newton")
        return p_b
    if np.linalg.norm(p_u) >= Delta:
        print("Dogleg: usando paso de Cauchy recortado")
        return Delta * p_u / np.linalg.norm(p_u)
    print("Dogleg: interpolando entre Cauchy y Newton")
    d = p_b - p_u
    a = np.dot(d, d)
    b = 2*np.dot(p_u, d)
    c = np.dot(p_u, p_u) - Delta**2
    discriminante = max(b*b - 4*a*c, 0.0)
    beta = (-b + np.sqrt(discriminante))/(2*a)
    return p_u + beta*d

def escribir_log(texto):
    with open(ruta_log, "a", encoding="utf-8") as f:
        f.write(texto + "\n")

class ResultadoDogleg:
    def __init__(self, x, fun):
        self.x = x
        self.fun = fun

if __name__ == "__main__":
    altura = 0.45
    rmin = 1.8
    rmax = 4.5

    num_puntos = tray_obj.puntos_necesarios()
    puntos3d = tray_obj.trayectoria_inicial(num_puntos, "circular")
    puntos2d = [[p[0], p[1]] for p in puntos3d]
    wp_in = np.ravel(puntos2d)

    print(f"\nOptimizando con DOGLEG en {len(puntos2d)} puntos de control (2D)")
    
    with open(ruta_historial, mode='w', newline='') as file:
        writer = csv.writer(file)
        encabezados = ["Evaluacion", "Costo", "Puntaje_vision", "restriccion_area", "Penalizacion_dmin", "Penalizacion_dmax", "Distancia_vuelo", "Tiempo_Vuelo"]
        for i in range(len(wp_in)//2):
            encabezados.extend([f"x{i}", f"y{i}", f"z{i}", f"yaw{i}"])
        writer.writerow(encabezados)

    N = len(wp_in)
    H = hessiana_inicial(N)
    Delta = 0.5
    max_iter = 4
    tol_grad = 1e-2

    with open(ruta_log, "w", encoding="utf-8") as f:
        f.write("Registro de optimzación de Dogleg\n")
        
    fitness_historial.append(fun_costo(wp_in))
    g = gradiente(fun_costo, wp_in)

    for k in range(max_iter):
        print("\n")
        print(f"ITERACIÓN DOGLEG {k+1}")
        escribir_log("")
        escribir_log(f"ITERACIÓN DOGLEG {k+1}")

        if np.linalg.norm(g) < tol_grad:
            print("\nConvergencia alcanzada.")
            break

        print("\nNorma del gradiente:", np.linalg.norm(g))
        escribir_log(f"Norma gradiente : {np.linalg.norm(g):.6f}")

        alpha = 0.05
        costo0 = fun_costo(wp_in)
        costo1 = fun_costo(wp_in - alpha * g)

        print("\nCosto actual :", costo0)
        print("Costo nuevo  :", costo1)
        escribir_log(f"Costo actual      : {costo0:.6f}")
        escribir_log(f"Costo nuevo       : {costo1:.6f}")

        pC = paso_cauchy(g, H)
        p_newton = paso_newton(H, g)
        p_dog = dogleg(g, H, Delta)

        print("\nRadio de confianza:", Delta)
        print("\nEvaluando paso Dogleg\n")

        costo_actual = fun_costo(wp_in)
        wp_nuevo = wp_in + p_dog
        costo_nuevo = fun_costo(wp_nuevo)

        print("\nCosto actual :", costo_actual)
        print("Costo nuevo  :", costo_nuevo)

        if abs(costo_actual - costo_nuevo) < 1e-3:
            print("\nCambio de costo muy pequeño. Convergencia.")
            break

        reduccion_real = costo_actual - costo_nuevo
        reduccion_predicha = -(g @ p_dog + 0.5 * p_dog @ H @ p_dog)

        if abs(reduccion_predicha) < 1e-12:
            rho = 0.0
        else:
            rho = reduccion_real / reduccion_predicha

        escribir_log(f"rho               : {rho:.6f}")

        DELTA_MIN = 0.10
        DELTA_MAX = 5.0
        if rho < 0.25:
            Delta = max(DELTA_MIN, 0.25 * Delta)
        elif rho > 0.75 and abs(np.linalg.norm(p_dog) - Delta) < 1e-6:
            Delta = min(2.0 * Delta, DELTA_MAX)

        escribir_log(f"Radio confianza   : {Delta:.6f}")

        if rho > 0.10:
            print("\n✓ Paso aceptado")
            escribir_log("Paso: ACEPTADO")
            wp_in = wp_nuevo
            fitness_historial.append(costo_nuevo)
            g_nuevo = gradiente(fun_costo, wp_in)
            s = p_dog
            y = g_nuevo - g
            if s @ y > 1e-8:
                H = actualizar_hessiana_bfgs(H, s, y)
                if np.linalg.cond(H) > 1e8:
                    H = np.eye(len(g))
            g = g_nuevo
            
    print("RESULTADO FINAL")
    costo_final = fun_costo(wp_in)
    
    escribir_log("\nRESULTADO FINAL")
    escribir_log(f"Costo final      : {costo_final:.6f}")

    # Guardar CSV resultado óptimo
    ty_opt = np.reshape(wp_in, (-1, 2)).tolist()
    wp_final = []
    wp_guardar_final = []
    for i, xy in enumerate(ty_opt):
        z = altura
        x = xy[0]
        y = xy[1]
        yaw = mt.atan2(1.0 - y, 1.0 - x)
        wp_final.append([x, y, z])
        wp_guardar_final.extend([x, y, z, yaw])
        
    with open(ruta_historial, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["OPTIMO", costo_final, "", "", "", "", "", ""] + wp_guardar_final)

    # Gráficas finales con la misma estructura que Optimizador.py
    resultado_mock = ResultadoDogleg(x=wp_in, fun=costo_final)
    graficas_finales(costos, resultado_mock, ruta_convergencia, ruta_base, altura, rmin, rmax)

    # Simulación final Webots
    diccionario = {"waypoints": wp_final}
    with open(rt_mision, "w") as f:
        json.dump(diccionario, f)
    
    comando = [webots_path, rt_mundo]
    print("SIMULACION FINAL")
    subprocess.run(comando)