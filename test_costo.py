import numpy as np
import math as mt
from trayectoria import control_trayectoria
from Optimizador import fun_costo

tray_obj = control_trayectoria()
num_puntos = tray_obj.puntos_necesarios()
puntos = tray_obj.puntos_trayectoria_circular(num_puntos)
puntos4d = []

for i in range(len(puntos)):
    x = puntos[i][0]
    y = puntos[i][1]
    z = puntos[i][2]
    dy = 1.0 - y
    dx = 1.0 - x
    yaw = mt.atan2(dy, dx)
    puntos4d.append([x, y, z, yaw])

puntos4d = np.array(puntos4d)
wp_in = np.ravel(puntos4d)

print("EVALUANDO COSTO BASE")
costo1 = fun_costo(wp_in)

# Perturbar el primer X en 5%
wp_in2 = np.copy(wp_in)
wp_in2[0] = wp_in2[0] * 1.05

print("\nEVALUANDO COSTO PERTURBADO")
costo2 = fun_costo(wp_in2)

print(f"\nCosto 1: {costo1}")
print(f"Costo 2: {costo2}")
print(f"Diferencia: {costo2 - costo1}")
