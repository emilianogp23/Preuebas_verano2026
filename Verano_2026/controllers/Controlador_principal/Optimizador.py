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
    # 1. Reconstruir a 3D a partir del vector p
    wp3d = np.reshape(p, (-1, 3)).tolist()
    v=0.0
    wp = []
    for w in wp3d:
        x = w[0]
        y = w[1]
        z = w[2]
        
        # Calcular el yaw para que siempre apunte al objeto en (1, 1)
        dx = 1.0 - x
        dy = 1.0 - y
        yaw = mt.atan2(dy, dx)
        
        if z<0.4:
            v+=10000*(0.4-z)**2
            z=0.4
        elif z>2.5:
            v+=10000*(z-2.5)**2
            z=2.5

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
    peso=3.2
    ptj=peso*puntaje
    
    # Calcular distancia teórica de la ruta (sin ruido de simulación)
    distancia_teorica = 0.0
    segmentos=[]
    for i in range(len(wp)-1):
        distancia_teorica += mt.sqrt((wp[i+1][0]-wp[i][0])**2 + (wp[i+1][1]-wp[i][1])**2)
        segmentos.append(mt.sqrt((wp[i+1][0]-wp[i][0])**2 + (wp[i+1][1]-wp[i][1])**2))
    # Sumar el tramo final al inicio para cerrar el circuito
    distancia_teorica += mt.sqrt((wp[0][0]-wp[-1][0])**2 + (wp[0][1]-wp[-1][1])**2)
    segmentos.append(mt.sqrt((wp[0][0]-wp[-1][0])**2 + (wp[0][1]-wp[-1][1])**2))
    medida=distancia_teorica/len(segmentos)
    penalizacion_distancia=0.0
    for seg in segmentos:
        d1=seg-medida
        d2=(d1**2)*10 # Penalización elástica (cuadrática)
        penalizacion_distancia+=d2

    # El costo ahora se basa en la distancia teórica (suave) en lugar del tiempo simulado (ruidoso)
    costo = distancia_teorica - ptj+penalizacion_distancia
    co=0.0
    for w in wp:
        w1=w[0]
        w2=w[1]
        w3=w[2]
        d=mt.sqrt((w1-1.0)**2+(w2-1.0)**2)
        if d<1.2:
            co += 1000.0 * (1.2 - d)**2
        elif d>5.5:
            co += 1000.0 * (d - 5.5)**2
        if w3>2.5 or w3<0.4:
            co+=1000.0*(w3-0.3)
    costo=costo+co+v
    costos.append(costo)

    #tiempo de vuelo, puntaje 
    print(f"Tiempo de vuelo: {tiempo_total:.2f} , Distancia teorica: {distancia_teorica:.2f} , puntaje: {puntaje:.1f}, costo: {costo:.2f} , |costo penalizacion: {penalizacion_distancia:.2f} , |puntos obtenidos: {ptj:.2f}|")
  

    x_vals = [w[0] for w in wp]
    y_vals = [w[1] for w in wp]
    x_vals.append(x_vals[0])
    y_vals.append(y_vals[0])

    plt.figure(figsize=(6,6))
    plt.plot(x_vals, y_vals, marker='x', color='blue', label='Ruta')
    for x, y in zip(x_vals[:-1], y_vals[:-1]):
        plt.text(x, y, f"({x:.1f}, {y:.1f})", fontsize=8, ha='left', va='bottom', color='black')

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
    nums_rand=[]
    n=1.7
    
   
    for i in range(len(puntos)):
        x=puntos[i][0]
        y=puntos[i][1]
        z=puntos[i][2]
        
        nums_rand=np.random.uniform(0.0,n,size=2)
       
        if i%2==0:
            x+=nums_rand[0]
            y+=nums_rand[1]
        
        # Solo agregamos x, y, z. El yaw se calcula en fun_costo
        puntos_opt.append([x, y, z])

    puntos3d=np.array(puntos_opt)
    wp_in=np.ravel(puntos3d)
    

    N = len(wp_in)
    simplex_inicial = np.zeros((N + 1, N))
    simplex_inicial[0] = wp_in
    radio_busqueda = 0.5 
    for i in range(N):
        simplex_inicial[i+1] = np.copy(wp_in)
        if i %4 ==3:
            simplex_inicial[i+1][i] += 0.0
        else:
            simplex_inicial[i+1][i] += 1.5

    print(f" optimizando... ")
    resultado=minimize(fun_costo, wp_in, method="Nelder-Mead", options={'maxiter':250, 'disp':True, 'initial_simplex': simplex_inicial, 'adaptive': True, 'xatol': 0.5, 'fatol': 1.0})
    print("\n")
    print(f"RESULTADO:{np.reshape(resultado.x,(-1,3))}")
    

    plt.plot(range(1, len(costos) + 1), costos, marker='o')
    plt.xlabel("Iteraciones")
    plt.ylabel("Costo (Error)")
    plt.title("Convergencia")
    plt.grid(True)
    plt.savefig(ruta_convergencia)
    plt.show()

    ty_opt=np.reshape(resultado.x,(-1,3)).tolist()

    # Recalculamos el yaw final para guardar la misión completa en 4D
    wp_final = []
    for w in ty_opt:
        x = w[0]
        y = w[1]
        z = w[2]
        dx = 1.0 - x
        dy = 1.0 - y
        yaw = mt.atan2(dy, dx)
        wp_final.append([x, y, z, yaw])

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
    ax.add_patch(plt.Circle((1.0, 1.0), 1.2, color='red', fill=False, linestyle='--'))
    ax.add_patch(plt.Circle((1.0, 1.0), 5.5, color='green', fill=False, linestyle='--'))

    plt.xlim(-5, 7)
    plt.ylim(-5, 7)
    plt.grid(True)
    plt.legend()
    plt.title("Trayectoria Final Optimizada")
    ruta_trayectoria_final = os.path.join(ruta_base, "trayectoria_final.png")
    plt.savefig(ruta_trayectoria_final)
    plt.show()
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

# def fun_costo(p):
#     global contador 
#     contador+=1
#     print(f"ITERACION {contador}")
    
#     # NUEVO: Reconstruir a partir de 2D (x, y). La Z la dejamos fija en 0.85 (tu altura_vuelo)
#     wp2d = np.reshape(p, (-1, 2)).tolist()
#     wp = []
#     z_fija = 0.85 
#     for w in wp2d:
#         x = w[0]
#         y = w[1]
#         dx = 1.0 - x
#         dy = 1.0 - y
#         yaw = mt.atan2(dy, dx)
#         wp.append([x, y, z_fija, yaw])
    
#     # 2. Escribir 'mision.json'
#     diccionario={
#         "waypoints":wp
#     }
#     with open(ruta_mision, "w") as f:
#         json.dump(diccionario, f)
    
#     # 3. Llamar a Webots (subprocess)
#     comando=[webots_path, "--mode=fast","--no-rendering","--minimize", "--batch", ruta_mundo]
#     subprocess.run(comando)
    
#     # 4. Leer 'reporte_vuelo.json'
#     with open(ruta_reporte, "r") as f:
#         diccionario2=json.load(f)
#         tiempo_total=diccionario2["tiempo_total"]

#     # 5. Retornar el costo
#     puntaje=detector.imagenes_capturadas()
#     peso=2.0
#     ptj=peso*puntaje
    
#     # Calcular distancia teórica
#     distancia_teorica = 0.0
#     for i in range(len(wp)-1):
#         distancia_teorica += mt.sqrt((wp[i+1][0]-wp[i][0])**2 + (wp[i+1][1]-wp[i][1])**2)
#     distancia_teorica += mt.sqrt((wp[0][0]-wp[-1][0])**2 + (wp[0][1]-wp[-1][1])**2)
    
#     costo = distancia_teorica - ptj
#     co=0.0
#     for w in wp:
#         w1=w[0]
#         w2=w[1]
#         d=mt.sqrt((w1-1.0)**2+(w2-1.0)**2)
#         if d<1.2:
#             co += 1000.0 * (1.2 - d)**2
#         elif d>5.5:
#             co += 1000.0 * (d - 5.5)**2
#     costo=costo+co
#     costos.append(costo)

#     print(f"Distancia teorica: {round(distancia_teorica,2)} | puntaje YOLO: {round(puntaje,2)} | COSTO FINAL: {round(costo,2)}")

#     x_vals = [w[0] for w in wp]
#     y_vals = [w[1] for w in wp]
#     x_vals.append(x_vals[0])
#     y_vals.append(y_vals[0])

#     plt.figure(figsize=(6,6))
#     plt.plot(x_vals, y_vals, marker='x', color='blue', label='Ruta')
#     plt.plot(1.0, 1.0, marker='*', color='gold', markersize=15, label='Objeto') 
#     ax = plt.gca()
#     ax.add_patch(plt.Circle((1.0, 1.0), 1.2, color='red', fill=False, linestyle='--'))
#     ax.add_patch(plt.Circle((1.0, 1.0), 5.5, color='green', fill=False, linestyle='--'))

#     plt.xlim(-5, 7)
#     plt.ylim(-5, 7)
#     plt.grid(True)
#     plt.title(f"Iteración {contador} - Costo: {round(costo, 2)}")
#     plt.savefig(os.path.join(ruta_historia, f"iteracion_{contador}.png"))
#     plt.close()

#     return costo

# if __name__ == "__main__":
#     if os.path.exists(ruta_historia) is False:
#         os.makedirs(ruta_historia)
#     else:
#         shutil.rmtree(ruta_historia)
#         os.makedirs(ruta_historia)
        
#     num_puntos=tray_obj.puntos_necesarios()
#     puntos=tray_obj.puntos_trayectoria_circular(num_puntos)
    
#     # NUEVO: Guardar solo X y Y (16 dimensiones en total)
#     puntos2d=[]
#     for i in range(len(puntos)):
#         x=puntos[i][0]
#         y=puntos[i][1]
        
#         if i%2==0:
#             x+=1.5

#         puntos2d.append([x, y])

#     puntos2d=np.array(puntos2d)
#     wp_in=np.ravel(puntos2d)
    
#     N = len(wp_in)
#     simplex_inicial = np.zeros((N + 1, N))
#     simplex_inicial[0] = wp_in
#     radio_busqueda = 0.5 
#     for i in range(N):
#         simplex_inicial[i+1] = np.copy(wp_in)
#         simplex_inicial[i+1][i] += radio_busqueda
    
#     print(f" Optimizando con {int(len(wp_in)/2)} puntos de control (Solo en 2D = {len(wp_in)} variables) ")
    
#     # maxiter 150 es un buen límite razonable
#     resultado=minimize(fun_costo, wp_in, method="Nelder-Mead", options={'maxiter':100, 'disp':True, 'initial_simplex': simplex_inicial, 'adaptive': True, 'xatol': 0.1, 'fatol': 0.1})
    
#     print("\nRESULTADO OPTIMIZACIÓN:")
#     print(np.reshape(resultado.x,(-1,2)))
    
#     plt.plot(range(1, len(costos) + 1), costos, marker='o')
#     plt.xlabel("Iteraciones")
#     plt.ylabel("Costo (Error)")
#     plt.title("Convergencia del Algoritmo de Optimización")
#     plt.grid(True)
#     plt.savefig(ruta_convergencia)
#     print("¡Terminado! Revisa la gráfica de convergencia.")

    