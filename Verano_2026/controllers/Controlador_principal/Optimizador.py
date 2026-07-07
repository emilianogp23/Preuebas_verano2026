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
from graficas import Graficar_trayectoria


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
altura=0.52
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
    
    ptj=peso*puntaje
    
    # El costo ahora se basa en el puntaje, volumen e irregularidades (ruidos)
    costo = -ptj+h+penalizacion_distancia+penalizacion_suavidad
    
    costos.append(costo)

    #tiempo de vuelo, puntaje 
    print(f"Tiempo de vuelo: {tiempo_total:.2f} , Distancia teorica: {distancia_teorica:.2f} , puntaje: {puntaje:.1f}, costo: {costo:.2f}  , |puntos obtenidos: {ptj:.2f}|")
  
    with open(ruta_historial, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([contador, costo] + np.ravel(wp_xy).tolist())

    Graficar_trayectoria(puntos,contador,costo,ruta_historia,radio_min,radio_max)


    return costo

if __name__ == "__main__":

    altura=0.52

   
        
    num_puntos=tray_obj.puntos_necesarios()
    puntos=tray_obj.puntos_trayectoria_circular(num_puntos)
    
    puntos_opt=[]
    
    # Rellenamos el vector inicial en modo Cartesiano (x, y)
    # Rellenamos el vector inicial en modo Cartesiano (x, y)
    for i in range(len(puntos)):
        x=puntos[i][0]
        y=puntos[i][1]
        
        # Agregamos ruido aleatorio seguro para que la trayectoria inicie "extraña"
        if i%2==0:
            x += np.random.uniform(-0.3, 0.3)
            y += np.random.uniform(-0.3, 0.3)
            x += np.random.uniform(-0.3, 0.3)
            y += np.random.uniform(-0.3, 0.3)
            
        puntos_opt.append([x, y])
        puntos_opt.append([x, y])

    puntos2d=np.array(puntos_opt)
    wp_in=np.ravel(puntos2d)
    print(f" Optimizando con COBYLA (2 variables (x,y)/pto, altura fija en {altura})... ")
    
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
    resultado = minimize(fun_costo, x0=wp_in, method='COBYLA', constraints=restricciones, options={'maxiter': 80, 'disp': True})
    
    print("\n")
    print(f"RESULTADO FINAL (XY):\n{np.round(np.reshape(resultado.x, (-1, 2)), 2)}")
    print(f"RESULTADO FINAL (XY):\n{np.round(np.reshape(resultado.x, (-1, 2)), 2)}")
    
    # Graficar la mejor convergencia (costo mínimo hasta cada iteración)
    mejores_costos = np.minimum.accumulate(costos)
    plt.plot(range(1, len(mejores_costos) + 1), mejores_costos, marker='o', color='blue', label='Mejor Costo')
    plt.plot(range(1, len(costos) + 1), costos, color='lightgray', alpha=0.5, label='Costo Evaluado')
    plt.xlabel("Evaluaciones")
    # Graficar la mejor convergencia (costo mínimo hasta cada iteración)
    mejores_costos = np.minimum.accumulate(costos)
    plt.plot(range(1, len(mejores_costos) + 1), mejores_costos, marker='o', color='blue', label='Mejor Costo')
    plt.plot(range(1, len(costos) + 1), costos, color='lightgray', alpha=0.5, label='Costo Evaluado')
    plt.xlabel("Evaluaciones")
    plt.ylabel("Costo (Error)")
    plt.title("Convergencia del Optimizador")
    plt.legend()
    plt.title("Convergencia del Optimizador")
    plt.legend()
    plt.grid(True)
    plt.savefig(ruta_convergencia)
    # plt.show() # Opcional: mostrar la gráfica al final

    ty_opt=np.reshape(resultado.x, (-1, 2)).tolist()
    ty_opt=np.reshape(resultado.x, (-1, 2)).tolist()

    # Recalculamos a Cartesianas para guardar la misión completa en 3D
    wp_final = []

    for i, xy in enumerate(ty_opt):
        z = altura
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
    with open(rt_mision, "w") as f:
        json.dump(diccionario, f)
    
    # 3. Llamar a Webots (subprocess)
    video_output= os.path.join(ruta_historia, f"video_final_{contador}.mp4")
    comando=[webots_path,  rt_mundo]
    print("SIMULACION FINAL")
    subprocess.run(comando)



# # Variables ajustables (Métricas)
# peso_fotos = 5.0        # Subir esto si quieres que priorice tomar MÁS fotos
# peso_distancia = 0.5    # Penaliza rutas innecesariamente largas
# peso_giros = 2.0        # (Si implementas cálculo de ángulos)

# # Tu costo final mejorado
# costo = -(peso_fotos * puntaje) + h + penalizacion_distancia + penalizacion_suavidad + (peso_distancia * distancia_teorica)

################################3333

# Claro que sí! Aquí tienes la explicación paso a paso y el código exacto de cómo implementar cada una de estas mejoras dentro de tu archivo Optimizador.py.

# Todas estas modificaciones van dentro de tu función fun_costo(waypointsrt).

# 1. Implementar el castigo por Distancia Teórica y Tiempo de Vuelo
# Ya estás calculando la distancia_teorica (alrededor de la línea 98-105) y ya estás extrayendo el tiempo_total del .json (alrededor de la línea 166). Solo falta agregarlos al cálculo del costo final.

# Cómo implementarlo: Ubícate donde calculas el costo final (alrededor de la línea 174, justo después de calcular puntaje = detector.imagenes_capturadas()). Vas a cambiar la forma en la que calculas el costo.

# Reemplaza tus líneas actuales del cálculo de costo por estas:

# python
# # 5. Retornar el costo
#     puntaje = detector.imagenes_capturadas()
    
#     # PESOS (Puedes ajustar estos valores para darle más importancia a una cosa u otra)
#     peso_fotos = 5.0        # Antes era 3.2. Lo subimos para priorizar las fotos
#     peso_dist = 0.5         # Penalización por cada metro extra recorrido
#     peso_tiempo = 0.2       # Penalización por cada segundo extra de vuelo
    
#     ptj = peso_fotos * puntaje
    
#     # El costo ahora incluye la distancia y el tiempo
#     costo = -ptj + h + penalizacion_distancia + penalizacion_suavidad + (peso_dist * distancia_teorica) + (peso_tiempo * tiempo_total)
    
#     costos.append(costo)
# ¿Qué hace esto? Al sumar la distancia y el tiempo al costo (recuerda que el optimizador busca el costo más bajo posible), el dron será castigado si toma una ruta innecesariamente larga o que tarde mucho, forzándolo a encontrar atajos eficientes.

# 2. Implementar la penalización por Giros Bruscos (Ángulos)
# Para evitar que el dron haga curvas de 90 grados o giros en "V" (los cuales gastan mucha batería y hacen que el dron frene de golpe), podemos calcular el ángulo entre cada 3 waypoints consecutivos. Si el giro es muy cerrado, sumamos una penalización.

# Cómo implementarlo: Agrega este bloque de código justo después de donde calculas la distancia_teorica (alrededor de la línea 106). Necesitarás la librería math (que ya tienes importada como mt).

# python
# # --- NUEVO: Penalización por giros bruscos (Ángulos) ---
#     penalizacion_giros = 0.0
#     # Agregamos el punto de inicio al principio y al final para evaluar el ciclo completo
#     wp_giros = [[-2.0, -2.0, 0.85]] + wp + [[-2.0, -2.0, 0.85]]
    
#     for i in range(len(wp_giros) - 2):
#         # Tomamos 3 puntos consecutivos (A, B, C)
#         A = wp_giros[i]
#         B = wp_giros[i+1]
#         C = wp_giros[i+2]
        
#         # Vectores AB y BC
#         AB = [B[0] - A[0], B[1] - A[1]]
#         BC = [C[0] - B[0], C[1] - B[1]]
        
#         # Magnitudes
#         mag_AB = mt.sqrt(AB[0]**2 + AB[1]**2)
#         mag_BC = mt.sqrt(BC[0]**2 + BC[1]**2)
        
#         if mag_AB > 0 and mag_BC > 0:
#             # Producto punto
#             dot_product = (AB[0] * BC[0]) + (AB[1] * BC[1])
#             # Coseno del ángulo (limitado entre -1 y 1 por errores de coma flotante)
#             cos_theta = max(-1.0, min(1.0, dot_product / (mag_AB * mag_BC)))
#             angulo_rad = mt.acos(cos_theta)
#             angulo_grados = mt.degrees(angulo_rad)
            
#             # Si el giro es mayor a 60 grados, lo penalizamos
#             if angulo_grados > 60.0:
#                 # Castigo progresivo: entre mayor sea el giro, peor el castigo
#                 penalizacion_giros += (angulo_grados - 60.0) * 2.0 
#     # --------------------------------------------------------
# Y luego, asegúrate de sumar penalizacion_giros al costo final y al costo de las posiciones inválidas (línea 151).

# python
# # En la validación de posiciones inválidas (línea ~151)
#     if h > 0:
#       ptj = peso_fotos * (-100.0)
#       costo = -ptj + h + penalizacion_distancia + penalizacion_suavidad + penalizacion_giros
#       # ...
# Y en el costo final:

# python
# # Costo final total 
#     costo = -ptj + h + penalizacion_distancia + penalizacion_suavidad + penalizacion_giros + (peso_dist * distancia_teorica) + (peso_tiempo * tiempo_total)
# 3. Modificar la Suavidad Continua (Distribución uniforme)
# Actualmente, solo penalizas si un salto es mayor a 2.5 metros. Vamos a cambiar esto para obligar al dron a que mantenga los waypoints distribuidos de forma similar.

# Cómo implementarlo: Modifica el bloque de Penalización por cambios bruscos (Suavidad) (alrededor de la línea 86) de la siguiente manera:

# python
# # Penalización por cambios bruscos (Suavidad)
#         if i > 0:
#             x_prev = wp_xy[i-1][0]
#             y_prev = wp_xy[i-1][1]
#             dist_prev = mt.sqrt((x - x_prev)**2 + (y - y_prev)**2)
            
#             # AHORA CASTIGAMOS DISTANCIAS LARGAS DE FORMA MÁS ESTRICTA
#             # En lugar de esperar a que sea 2.5m, si la distancia supera 1.5m ya empezamos a penalizar
#             if dist_prev > 1.5:
#                 penalizacion_suavidad += (dist_prev - 1.5) * 150.0
# Resumen de lo que lograrás con estos cambios:
# Dron más inteligente: Buscará rutas cortas y rápidas en lugar de solo "volar por volar".
# Curvas orgánicas: Al penalizar los ángulos > 60°, el optimizador acomodará los puntos formando circunferencias o elipses naturales alrededor del objeto.
# Distribución equitativa: Al bajar el límite de salto a 1.5 metros, los puntos no se acumularán todos de un lado dejando el otro lado vacío.
# ¿Quieres que aplique todas estas mejoras directamente a tu código Optimizador.py por ti, o prefieres copiar y pegarlas tú mismo para ir haciendo las pruebas?

# 12:15 PM



#Cambio a splines 

# El tambaleo y el hecho de que el dron se detenga en cada punto ocurre porque, en tu archivo Controlador_principal.py, estás utilizando la clase Generador.generar_tray().

# La matemática detrás de ese generador asume que quieres ir de Punto A a Punto B y detenerte. Calcula un polinomio que empieza con velocidad cero y termina con velocidad cero. Como recalculas esto para cada segmento, el dron frena totalmente antes de avanzar al siguiente.

# Para que la ruta sea 100% continua, curva y fluida, debemos crear una única trayectoria completa al principio (usando algo llamado "Spline") que atraviese todos tus waypoints sin detenerse, manteniendo la velocidad constante.

# Cómo solucionarlo:
# Vas a editar tu archivo Controlador_principal.py. Ve a la línea 190 (donde empieza el if indice_wp < len(puntos):).

# Vas a borrar desde esa línea hasta la línea 218 (justo antes del else:) y lo vas a reemplazar por este nuevo bloque de código:

# python
# if indice_wp < len(puntos):
        
#         if funcionando == False:
#             from scipy.interpolate import CubicSpline
            
#             # 1. Creamos la lista de puntos iniciando desde la posición ACTUAL del dron
#             distancias = [0.0]
#             puntos_limpios = [pos_act] 
            
#             # 2. Unimos todos los waypoints midiendo la distancia entre ellos
#             for i in range(len(puntos)):
#                 p_prev = puntos_limpios[-1]
#                 p_act = puntos[i][:3]  # Asegurar que solo tomamos X,Y,Z
                
#                 # Calculamos la distancia con el punto anterior
#                 d = mt.sqrt((p_act[0]-p_prev[0])**2 + (p_act[1]-p_prev[1])**2 + (p_act[2]-p_prev[2])**2)
                
#                 # Evitamos duplicados o puntos demasiado juntos (causan error matemático en el spline)
#                 if d > 0.01: 
#                     distancias.append(distancias[-1] + d)
#                     puntos_limpios.append(p_act)
                    
#             distancia_total_ruta = distancias[-1]
            
#             # Tiempo estimado para recorrer TODO el circuito a velocidad constante
#             t_vuelo = max(distancia_total_ruta / vel_dron, 1.0)
            
#             # 3. Generar el Spline Cúbico Continuo (la curva suave)
#             # Esto interpola todos los puntos en un solo trazo sin frenadas
#             spline_trayectoria = CubicSpline(distancias, puntos_limpios)
            
#             t_inicial = t_act
#             funcionando = True
        
#         # 4. Calcular dónde debería estar el dron en este instante de tiempo
#         t_tray = t_act - t_inicial
        
#         # Distancia teórica que el dron debió haber recorrido a velocidad constante
#         distancia_actual = min(t_tray * vel_dron, distancia_total_ruta)
        
#         # Evaluar la curva en esa distancia
#         pos_des = spline_trayectoria(distancia_actual)
#         x_des = float(pos_des[0])
#         y_des = float(pos_des[1])
#         z_des = float(pos_des[2])
#         w_des = normalizar_angulos(yd) 
        
#         # 5. Condición para terminar la simulación (cuando el tiempo total se acaba)
#         # Le damos 1 segundo extra para asegurar que llegó al último punto
#         if t_tray > t_vuelo + 1.0: 
#             indice_wp = len(puntos) # Esto forzará que pase al 'else' de abajo y apague motores
            
#     else:
# ¿Qué es lo que logramos con este cambio?
# Nunca se detiene: En lugar de decirle "ve al punto 2 y luego te digo qué hacer", le das el mapa completo al inicio.
# Usa Splines (CubicSpline): Esta función matemática traza curvas orgánicas a través de todos tus puntos. Las esquinas se redondearán automáticamente de forma suave.
# Velocidad Constante: El dron calculará qué tan lejos debe estar en cada milisegundo de simulación basándose en vel_dron. No habrá aceleración ni frenada en los puntos intermedios.
# Intenta pegar este cambio en tu Controlador_principal.py y correr la simulación, ¡notarás una diferencia gigante en cómo se mueve el dron en Webots!