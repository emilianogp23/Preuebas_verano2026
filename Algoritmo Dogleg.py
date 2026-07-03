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
        z = w[2] #Dejar fijo si se mueven demasiadas iteraciones
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

    # Paso de Cauchy verdadero
    p_u = paso_cauchy(g, H)

    # Paso de Newton
    p_b = paso_newton(H, g)

    # Caso 1: Newton cabe completo
    if np.linalg.norm(p_b) <= Delta:

        print("Dogleg: usando paso de Newton")

        return p_b

    # Caso 2: incluso Cauchy sale de la región
    if np.linalg.norm(p_u) >= Delta:

        print("Dogleg: usando paso de Cauchy recortado")

        return Delta * p_u / np.linalg.norm(p_u)

    # Caso 3: interpolación Dogleg

    print("Dogleg: interpolando entre Cauchy y Newton")

    d = p_b - p_u

    a = np.dot(d, d)

    b = 2*np.dot(p_u, d)

    c = np.dot(p_u, p_u) - Delta**2

    discriminante = b*b - 4*a*c

    discriminante = max(discriminante, 0.0)

    beta = (-b + np.sqrt(discriminante))/(2*a)

    return p_u + beta*d

if __name__ == "__main__":

    if not os.path.exists(ruta_historia):
        os.makedirs(ruta_historia)
    else:
        shutil.rmtree(ruta_historia)
        os.makedirs(ruta_historia)

    num_puntos = tray_obj.puntos_necesarios()
    puntos = tray_obj.puntos_trayectoria_circular(num_puntos)

    puntos3d = []

    for p in puntos:
        puntos3d.append([p[0], p[1], p[2]])

    puntos3d = np.array(puntos3d)
    wp_in = np.ravel(puntos3d)

    N = len(wp_in)

    print(f"\nOptimizando con {len(puntos3d)} puntos de control (3D)")

    # Inicialización (solo una vez)
    H = hessiana_inicial(N)
    Delta = 0.5

    max_iter = 20
    tol_grad = 1e-3

    for k in range(max_iter):

        print("\n")
        print(f"ITERACIÓN DOGLEG {k+1}")

        g = gradiente(fun_costo, wp_in)

        if np.linalg.norm(g) < tol_grad:

            print("\nConvergencia alcanzada.")
            break

        print("\nGradiente (por waypoint):")
        print(np.reshape(g, (-1,3)))

        print("\nGradiente completo:")
        print(g)

        print("\nNorma del gradiente:")
        print(np.linalg.norm(g))

        alpha = 0.05

        costo0 = fun_costo(wp_in)

        costo1 = fun_costo(wp_in - alpha * g)

        print("\nCosto actual :", costo0)
        print("Costo nuevo  :", costo1)

        if costo1 < costo0:
            print("\n✓ El gradiente apunta hacia descenso.")
        else:
            print("\n⚠ El gradiente NO produjo descenso.")

        pC = paso_cauchy(g, H)

        print("\nPaso de Cauchy:")
        print(np.reshape(pC, (-1,3)))

        print("\nNorma del paso:")
        print(np.linalg.norm(pC))

        p_newton = paso_newton(H, g)

        print("\nPaso de Newton:")
        print(np.reshape(p_newton, (-1,3)))

        print("\nNorma:")
        print(np.linalg.norm(p_newton))

        p_dog = dogleg(g, H, Delta)

        print("\nPaso Dogleg:")
        print(np.reshape(p_dog, (-1,3)))

        print("\nNorma:")
        print(np.linalg.norm(p_dog))

        print("\nRadio de confianza:", Delta)

        if np.linalg.norm(p_newton) <= Delta:
            print("El paso de Newton está dentro de la región de confianza.")
        else:
            print("El paso de Newton está fuera de la región de confianza.")

        print("\nEvaluando paso Dogleg...\n")

        costo_actual = fun_costo(wp_in)

        wp_nuevo = wp_in + p_dog

        costo_nuevo = fun_costo(wp_nuevo)

        print("\nCosto actual :", costo_actual)
        print("Costo nuevo  :", costo_nuevo)

        if abs(costo_actual - costo_nuevo) < 1e-3:

            print("\nCambio de costo muy pequeño.")

            print("Se considera que el algoritmo ha convergido.")

            break

        reduccion_real = costo_actual - costo_nuevo

        reduccion_predicha = -(g @ p_dog + 0.5 * p_dog @ H @ p_dog)

        print("\nReducción real:", reduccion_real)
        print("Reducción predicha:", reduccion_predicha)

        if abs(reduccion_predicha) < 1e-12:
            rho = 0.0
        else:
            rho = reduccion_real / reduccion_predicha

        print("\nrho =", rho)

        # Actualización del radio de confianza

        DELTA_MIN = 0.10
        DELTA_MAX = 5.0

        if rho < 0.25:

            Delta = max(DELTA_MIN, 0.25 * Delta)

        elif rho > 0.75 and abs(np.linalg.norm(p_dog) - Delta) < 1e-6:

            Delta = min(2.0 * Delta, DELTA_MAX)

        print("\nNuevo radio:", Delta)

        if rho > 0.10:

            print("\n✓ Paso aceptado")

            wp_in = wp_nuevo

            g_nuevo = gradiente(fun_costo, wp_in)

            s = p_dog

            y = g_nuevo - g

            if s @ y > 1e-8:

                H = actualizar_hessiana_bfgs(H, s, y)

                # Evitar que la Hessiana se vuelva numéricamente inestable
                if np.linalg.cond(H) > 1e8:

                    print("Hessiana mal condicionada. Reiniciando...")

                    H = np.eye(len(g))

                print("Hessiana actualizada mediante BFGS.")

            else:

                print("No se pudo actualizar la Hessiana.")

        print("\nPosición actual:")
        print(np.reshape(wp_in, (-1,3)))

        print("\nNorma gradiente:", np.linalg.norm(g))
        print("Radio:", Delta)
        print("Costo:", costo_actual)

    print("\n")
    print("="*60)
    print("RESULTADO FINAL")
    print("="*60)

    print(np.reshape(wp_in, (-1,3)))

    print("\nCosto final:")
    print(fun_costo(wp_in))

    print("\nNorma del gradiente:")
    print(np.linalg.norm(g))

    print("\nDimensión Hessiana:")
    print(H.shape)

    print("\nNorma Hessiana:")
    print(np.linalg.norm(H))

    plt.figure()
    plt.plot(range(1, len(costos)+1), costos, marker='o')
    plt.xlabel("Evaluación")
    plt.ylabel("Costo")
    plt.title("Costo durante las evaluaciones")
    plt.grid(True)
    plt.savefig(ruta_convergencia)
    plt.show()