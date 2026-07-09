
## Estructura del Proyecto

- `controllers/Controlador_principal/Optimizador.py`: Genera las misiones iniciales, llama a Webots , y optimiza la trayectoria 
- `controllers/Controlador_principal/trayectoria.py`: Contiene los algoritmos para generar las trayectorias iniciales (Circular, Cuadrada, Aleatoria).
- `controllers/Controlador_principal/yolo_detector.py`: Evalúa y califica las fotografías capturadas durante la misión.
- `controllers/Controlador_principal/Controlador_principal.py`: Archivo de control lógico y de movimiento del dron dentro de Webots 

Antes de lanzar el optimizador, puedes elegir qué trayectoria inicial probar.
En el archivo Optimizador es necesario elegir la trayectoria inicial, en la variable puntos 3d hay que seleccionar el tipo de trayectoria sera la inicial. 
    # Opciones disponibles: "circular", "cuadrada", "Random", "circulo_ruido", "cuadrado_ruido"
Al igual que en archivo Trayectoria hasta abajo en el main la varibale "tray" hay que modificarla y poner el nombre de la trayctoria deseada.


Puedes cambiar `"cuadrada"` por la opción que prefieras para ver cómo el algoritmo organiza la ruta.

## Ejecutar el código
Ejecutar el archivo de optimización directamente.

### Proceso de Ejecución
1. El script generará la trayectoria inicial solicitada.
2. Comenzará a probar rutas mediante simulaciones iterativas en Webots. 
3. Se imprimirán en consola datos por cada evaluación: `Tiempo de vuelo`, `Distancia teórica`, `Puntaje de fotos` y `Costo final`.
4. Si la ruta entra en áreas prohibidas, rebasa los límites exteriores o los saltos entre puntos son muy bruscos, la consola mostrará **`Posición inválida`** junto con el costo.
5. Al finalizar las evaluaciones, el dron ejecutará una simulación final abriendo la interfaz de Webots visiblemente.
6. Todos los resultados, históricos de iteraciones y gráficas de convergencia quedarán en la carpeta `Resultados/`.

## Función de Costo

Si se desea cambiar el comportamiento del optimizador (que priorice capturar fotos, que intente volar más rápido o que sea más estricto con la distancia), se pueden ajustar los pesos en `Optimizador.py` dentro de la función `fun_costo`:

- `peso_fotos`: Prioriza la captura de objetos bien centrados y dimensionados.
- `peso_dist`: Penaliza rutas excesivamente largas.
- `peso_tiempo`: Penaliza las simulaciones lentas.
- *Y las distancias de los límites y umbrales (radio_min, radio_max, suavidad).*
