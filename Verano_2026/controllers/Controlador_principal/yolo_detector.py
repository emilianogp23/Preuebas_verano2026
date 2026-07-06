import cv2
import numpy as np
import math
# pyrefly: ignore [missing-import]
from ultralytics import YOLO
import os 
import glob

class YoloDetector:
    def __init__(self, model_path="yolov8n.pt", window_name="Drone Camera"):
        dir_actual = os.path.dirname(os.path.abspath(__file__))
        if not os.path.isabs(model_path):
            model_path_abs = os.path.join(dir_actual, model_path)
            if os.path.exists(model_path_abs):
                model_path = model_path_abs
            else:
                model_path_base = os.path.abspath(os.path.join(dir_actual, "..", "..", model_path))
                if os.path.exists(model_path_base):
                    model_path = model_path_base
        self.model = YOLO(model_path)
        self.window_name = window_name
        self.clases_aceptadas = ["person", "bottle", "cup", "vase", "fire hydrant", "sports ball"]
        #cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
        self.ruta_fotos = os.path.join(dir_actual, "fotos_capturadas")
        self.ruta_debug=os.path.join(dir_actual,"fotos_debug")
        os.makedirs(self.ruta_debug, exist_ok=True)
        
        for f in os.listdir(self.ruta_fotos):
            os.remove(os.path.join(self.ruta_fotos, f))
        for f in os.listdir(self.ruta_debug):
            os.remove(os.path.join(self.ruta_debug, f))

        

    def imagenes_capturadas(self):
        
        self.mejores_puntajes=[]
        img=os.path.join(self.ruta_fotos,f"*.png")
        nombres=glob.glob(img)
        
        if len(nombres)==0:
            print("No se encontraron imagenes")
            return -100.0
        for n in nombres:
            if not os.path.exists(n) or os.path.getsize(n) == 0:
                print(f"Advertencia: El archivo {n} está vacío o no existe.")
                continue
            img_np=cv2.imread(n)
            if img_np is None:
                print(f"Advertencia: No se pudo leer la imagen {n}")
                continue
            img_width=img_np.shape[1]
            img_height=img_np.shape[0]
            puntaje=self.process_image(img_np, img_width, img_height, 0.87,n)
            
            # Penalizar fuertemente si no hay detecciones en la foto
            if puntaje == 0.0:
                puntaje = -10.0
                
            self.mejores_puntajes.append(puntaje)
        
        if len(self.mejores_puntajes) > 0:
            puntaje_final = sum(self.mejores_puntajes)/len(self.mejores_puntajes)
        else:
            puntaje_final = -100.0
            
        return puntaje_final

    def process_image(self, img_data, img_width, img_height, fov,n_foto):
        img_debug=img_data.copy()
        if  img_data is None:
            return 0.0 
        img_bgr=img_data
        # Inferencia YOLO (silenciada para no llenar la consola)
        results = self.model(img_bgr, verbose=False)
        puntaje=0
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

                # Penalizacion por bordes 
                borde_vertical=10
                borde_horizontal=100
                punto_medio_img=img_width/2
                punto_medio_obj=(x2+x1)/2
                p_medio1=punto_medio_img+35
                p_medio2=punto_medio_img-35
                puntaje_act=0


                altura = img_height
                altura_obj = y2 - y1
                porc = (altura_obj / altura) * 100.0
                
                # Evaluacion de altura de objeto (Deseado: 60% de la altura de la imagen)
                error_altura = abs(porc - 80.0)
                # 5.0 puntos si es perfecto (error 0). 
                # Si el error es 40% (ej. el objeto ocupa 20% o 100%), restamos 10 puntos -> puntaje -5.0
                puntaje_altura = 5.0 - (error_altura / 40.0) * 10.0
                    
                # Evaluacion de centrado horizontal 
                error_centrado = abs(punto_medio_obj - punto_medio_img)
                max_err_h = img_width / 2.0
                # 5.0 puntos si está en el centro.
                # Si está en el borde (error_centrado = max_err_h), restamos 15 puntos -> puntaje -10.0
                puntaje_centrado = 5.0 - (error_centrado / max_err_h) * 15.0

                # Evaluacion de centrado vertical
                punto_medio_img_v = img_height / 2.0
                punto_medio_obj_v = (y2 + y1) / 2.0
                error_centrado_v = abs(punto_medio_obj_v - punto_medio_img_v)
                max_err_v = img_height / 2.0
                puntaje_centrado_v = 5.0 - (error_centrado_v / max_err_v) * 15.0
                
                # Sumar los 3 puntajes (puede ser negativo si está muy mal encuadrado)
                puntaje_act = puntaje_altura + puntaje_centrado + puntaje_centrado_v
                
                # Penalización extra si de plano toca los márgenes (casi fuera de cámara)
                margen_h = img_width * 0.05
                margen_v = img_height * 0.05
                if x1 <= margen_h or x2 >= img_width - margen_h or y1 <= margen_v or y2 >= img_height - margen_v:
                    puntaje_act -= 10.0

                if puntaje_act > puntaje or puntaje == 0:
                    puntaje = puntaje_act

                cv2.rectangle(img_debug, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.circle(img_debug, (int(punto_medio_obj), int(punto_medio_obj_v)), 5, (0, 0, 255), -1)
                cv2.circle(img_debug,(int(img_width/2), int(img_height/2)),5,(255,0,0),-1)
                cv2.putText(img_debug, f"Score: {puntaje_act:.2f}", (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.imwrite(os.path.join(self.ruta_debug,os.path.basename(n_foto)), img_debug)
            
                
                

                # if y1>borde_vertical and y2<img_height-borde_vertical:
                #     foto_buena=True
                #     altura=img_height
                #     altura_obj=y2-y1
                #     porc=(altura_obj/altura)*100
                    
                #     # Evaluacion de altura de objeto
                #     error_altura = abs(porc - 70.0)
                #     # Tolerancia más amplia
                #     puntaje_altura = max(0.0, 5.0 - (error_altura / 10.0))
                #     puntaje_act += puntaje_altura
                     
                #     # Evaluacion de centrado horizontal 
                #     error_centrado = abs(punto_medio_obj - punto_medio_img)
                #     puntaje_centrado = max(0.0, 5.0 - (error_centrado / (img_width / 15.0)))
                #     puntaje_act += puntaje_centrado
                    
                #     if puntaje_act>puntaje:
                #         puntaje=puntaje_act
                
                # else:
                #     foto_buena=False

        return puntaje

             
                
                
       