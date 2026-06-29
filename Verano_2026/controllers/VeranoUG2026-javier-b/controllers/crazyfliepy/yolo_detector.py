import cv2
import numpy as np
import math
# pyrefly: ignore [missing-import]
from ultralytics import YOLO
import os 
import glob

class YoloDetector:
    def __init__(self, model_path="yolov8n.pt", window_name="Drone Camera"):
        self.model = YOLO(model_path)
        self.window_name = window_name
        self.clases_aceptadas = ["person", "bottle", "cup", "vase", "fire hydrant", "sports ball"]
        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
        self.ruta_fotos="/home/jpirmz/Documents/PR_Bebop/Pruebas_verano2026/Verano_2026/controllers/Controlador_principal/fotos_capturadas"

    def imagenes_capturadas(self):
        img=os.path.join(self.ruta_fotos,f"*.png")
        nombres=glob.glob(img)
        for n in nombres:
            img_np=cv2.imread(n)
            img_width=img_np.shape[1]
            img_height=img_np.shape[0]
            self.process_image(img_np, img_width, img_height, 0.87)
            
        return nombres

    def process_image(self, img_data, img_width, img_height, fov):
        if  img_data is None:
            return 0.0 
        img_bgr=img_data
        # Inferencia YOLO (silenciada para no llenar la consola)
        results = self.model(img_bgr, verbose=False)
       
        for r in results:
            for box in r.boxes:
                # Coordenadas Bounding box
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                conf = round(float(box.conf[0]), 2)

                #Relacion entre altura objeto y foto 
                #Centrado horizontal 
                
                # Ignorar detecciones con baja confianza (menor al 40%)
                if conf < 0.4:
                    continue
                    
                cls = int(box.cls[0])
                class_name = self.model.names[cls]
                
                # Filtrar por clases (ignorar sillas, mesas, monitores, etc)
                # "person" es el humano. Las otras son clases típicas con las que YOLO confunde a un barril
                if class_name not in self.clases_aceptadas:
                    continue
      
                # Estimacion de tamaño y distancia
                bbox_height = float(y2 - y1)
                if bbox_height <= 0:
                    bbox_height = 1.0 # Prevenir division por cero
                
                # Alturas estimadas reales (metros)
                if class_name == "person":
                    real_height = 1.70 # Humano
                else:
                    real_height = 1.0 
                    
                # Calculo de distancia usando Pinhole model
                dist_estimada = (real_height * img_height) / (bbox_height * 2 * math.tan(fov/2))

                #Penalizacion por bordes 
                borde_vertical=10
                borde_horizontal=100

                if y1>borde_vertical and y2<img_height-borde_vertical and x1>borde_horizontal and x2<img_width-borde_horizontal:
                    foto_buena=True
                    puntaje=0.0
                    altura=img_height
                    altura_obj=y2-y1
                    porc=(altura_obj/altura)*100
                    if porc >=80 and porc <=90:
                        puntaje += 5
                        punto_medio_img=img_width/2
                        punto_medio_obj=(x2-x1)/2
                        p_medio=punto_medio_img+15
                        
                        if punto_medio_obj >=p_medio and punto_medio_obj <=p_medio:
                            puntaje +=5
                        else:
                            puntaje -=5
                        
                        
                    else:
                        puntaje=0.0
                        

                else:
                    foto_buena=False
                    puntaje =0.0

             
                
                
       