import matplotlib.pyplot as plt
import os 
import numpy as np


def Graficar_trayectoria(puntos,contador,costo,path,rmin,rmax):
    wp=puntos

    x_vals = [w[0] for w in wp]
    y_vals = [w[1] for w in wp]
    z_vals=[w[2] for w in wp]

    x_vals.append(x_vals[0])
    y_vals.append(y_vals[0])
    z_vals.append(z_vals[0])
    
    plt.figure(figsize=(6,6))
    plt.plot(x_vals, y_vals, marker='x', color='blue', label='Ruta')
    lista="Coordenadas: \n"
    ax = plt.gca()
    for i, (x, y, z) in enumerate(zip(x_vals[:-1], y_vals[:-1], z_vals[:-1])):
        #plt.text(x+0.15, y+0.15, f"({x:.1f}, {y:.1f},{z:.1f})", fontsize=8, ha='right', va='center', color='black')
        plt.text(x+0.15, y+0.15, f"P{i}", fontsize=8, color='black',)
        lista+=f"P{i}: ({x:.1f}, {y:.1f},{z:.1f})\n"
    lista+="\n"
    caja=dict(boxstyle='round', facecolor='white', alpha=0.5,edgecolor='black')
    ax.text(1.05,0.95,lista,transform=ax.transAxes,fontsize=10,verticalalignment='top',bbox=caja)
        

    plt.plot(1.0, 1.0, marker='*', color='gold', markersize=15, label='Objeto') 
    
    ax.add_patch(plt.Circle((1.0, 1.0), rmin, color='red', fill=False, linestyle='--'))
    ax.add_patch(plt.Circle((1.0, 1.0), rmax, color='green', fill=False, linestyle='--'))
    
    txt=f"Evaluacion: {contador} |Costo: {costo:.2f}"
    ax.text(0.05,0.05,txt,transform=ax.transAxes,fontsize=10,verticalalignment='bottom',bbox=caja)

    plt.xlim(-5, 7)
    plt.ylim(-5, 7)
    plt.grid(alpha=0.3,ls="--", color='gray')
    plt.legend(loc='upper right')
    plt.title(f"Grafica evaluacion: {contador}")
    plt.savefig(os.path.join(path, f"Evaluacion_{contador}.png"),dpi=300,bbox_inches='tight')
    plt.close()


def graficas_finales(costos,resultado,ruta_convergencia,ruta_base,altura,rmin,rmax):
    costo=resultado.fun

    # Graficar la mejor convergencia (costo mínimo hasta cada iteración)
    mejores_costos = np.minimum.accumulate(costos)
    plt.plot(range(1, len(mejores_costos) + 1), mejores_costos, marker='o', color='blue', label='Mejor Costo')
    plt.plot(range(1, len(costos) + 1), costos, color='lightgray', alpha=0.5, label='Costo Evaluado')
    # plt.ylim(bottom=min(costos)-10, top=20)
    plt.xlabel("Evaluaciones")
    plt.ylabel("Costo")
    plt.title("Convergencia del Optimizador")
    plt.legend()
    plt.grid(True)
    plt.savefig(ruta_convergencia)
    # plt.show() # Opcional: mostrar la gráfica al final

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

    plt.figure(figsize=(6,6))
    plt.plot(x_opt, y_opt, marker='x', color='blue', label='Ruta')
    lista="Coordenadas (x, y, z, yaw): \n"
    ax = plt.gca()
    for i, (x, y) in enumerate(zip(x_opt[:-1], y_opt[:-1])):
        yaw = np.arctan2(1.0 - y, 1.0 - x)
        #plt.text(x+0.15, y+0.15, f"({x:.1f}, {y:.1f},{z:.1f})", fontsize=8, ha='right', va='center', color='black')
        plt.text(x+0.15, y+0.15, f"P{i}", fontsize=8, color='black',)
        lista+=f"P{i}: ({x:.1f}, {y:.1f}, {altura:.1f}, {yaw:.1f})\n"
    lista+="\n"
    caja=dict(boxstyle='round', facecolor='white', alpha=0.5,edgecolor='black')
    ax.text(1.05,0.95,lista,transform=ax.transAxes,fontsize=10,verticalalignment='top',bbox=caja)
        

    plt.plot(1.0, 1.0, marker='*', color='gold', markersize=15, label='Objeto') 
    
    ax.add_patch(plt.Circle((1.0, 1.0), rmin, color='red', fill=False, linestyle=':'))
    ax.add_patch(plt.Circle((1.0, 1.0), rmax, color='blue', fill=False, linestyle=':'))
    
    txt=f"Ruta optimizada \nCosto: {costo:.2f}"
    ax.text(0.05,0.05,txt,transform=ax.transAxes,fontsize=10,verticalalignment='bottom',bbox=caja)

    plt.xlim(-5, 7)
    plt.ylim(-5, 7)
    plt.grid(alpha=0.3,ls="--", color='gray')
    plt.legend(loc='upper right')
    plt.title(f"Ruta optimizada")
    plt.savefig(os.path.join(ruta_base, f"Ruta optimizada.png"),dpi=300,bbox_inches='tight')
    plt.show()
    
    pass 
