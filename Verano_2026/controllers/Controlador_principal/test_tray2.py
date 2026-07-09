import math as mt
def trayectoria_cuadrada(numero_puntos):
    trayectoria = []
    L = 4.5
    posx_ini = -2.0
    posy_ini = -2.0
    altura_vuelo = 0.5
    corners = [
        (posx_ini, posy_ini),
        (posx_ini + L, posy_ini),
        (posx_ini + L, posy_ini + L),
        (posx_ini, posy_ini + L)
    ]
    
    perimeter = 4 * L
    step = perimeter / numero_puntos
    
    for i in range(numero_puntos):
        d = i * step
        if d < L:
            x = corners[0][0] + d
            y = corners[0][1]
        elif d < 2 * L:
            x = corners[1][0]
            y = corners[1][1] + (d - L)
        elif d < 3 * L:
            x = corners[2][0] - (d - 2 * L)
            y = corners[2][1]
        else:
            x = corners[3][0]
            y = corners[3][1] - (d - 3 * L)
        trayectoria.append([x, y, altura_vuelo])
        
    return trayectoria

for p in trayectoria_cuadrada(14):
    print(round(p[0], 2), round(p[1], 2))
