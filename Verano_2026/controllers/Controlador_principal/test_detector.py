import os
from yolo_detector import YoloDetector

detector = YoloDetector()
puntaje = detector.imagenes_capturadas()
print(f"Puntaje total: {puntaje}")
print(f"Mejores puntajes: {detector.mejores_puntajes}")
