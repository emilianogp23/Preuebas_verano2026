Aquí ya se tiene la implementación de la reconstrucción en 3D usando colmap para realizar el modelo.
Está configurado para 200 fotografías, pero eso se puede modificar
Si la simulación se traba al inicio, elimina la carpeta fotos_capturadas y reinicia la simulación.
En colmap ve a File > import model y en colmap_output selecciona sparse > 0.

También se modificó parte del controlador principal para incluir colmap y del mundo en webots para la toma de fotografías y el reconocimiento del modelo.
