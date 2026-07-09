from trayectoria import control_trayectoria
import math as mt
t = control_trayectoria()
pts = t.trayectoria_cuadrada(24)
for p in pts:
    print(p)
