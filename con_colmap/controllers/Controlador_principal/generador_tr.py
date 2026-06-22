import numpy as np 

class Generador:
   
    def generar_tray(p0, pf, t):
        p0 = np.array(p0, dtype=float)
        pf = np.array(pf, dtype=float)
        t = float(t)

        a0 = p0
        a1 = np.zeros_like(p0)

        a2 = (3.0 * (pf - p0)) / (t**2)
        a3 = (-2.0 * (pf - p0)) / (t**3)

        return a0, a1, a2, a3
    
    def eval(a0, a1, a2, a3, t, T):
        t = float(t)
        t=min(t,float(T))

        pos = a0 + a1 * t + a2 * (t**2) + a3 * (t**3)
        if t>=float(T):
            vel=np.zeros(4)
        else:
            
            vel = a1 + 2 * a2 * t + 3 * a3 * (t**2)

        return pos, vel
    
    def generar_ruta(x_min, x_max, y_min, y_max, separacion,altura):
        waypoints = []
        y_actual = y_min
        ir_derecha = True

        while y_actual <= y_max:
            if ir_derecha:
                waypoints.append([x_min, y_actual, altura, 0.0])
                waypoints.append([x_max, y_actual, altura, 0.0])
            else:
                waypoints.append([x_max, y_actual, altura, 0.0])
                waypoints.append([x_min, y_actual, altura, 0.0])
            ir_derecha = not ir_derecha
            y_actual += separacion

        return waypoints





    