import os
import cv2
from yolo_detector import YoloDetector

detector = YoloDetector()

for f in os.listdir(detector.ruta_fotos):
    print(f"\n--- Analizando {f} ---")
    img_path = os.path.join(detector.ruta_fotos, f)
    img_np = cv2.imread(img_path)
    if img_np is None: continue
    
    img_height, img_width = img_np.shape[:2]
    print(f"Dimensiones de imagen: {img_width}x{img_height}")
    
    results = detector.model(img_np, verbose=False)
    for r in results:
        for box in r.boxes:
            conf = round(float(box.conf[0]), 2)
            if conf < 0.4: continue
            
            cls = int(box.cls[0])
            class_name = detector.model.names[cls]
            if class_name not in detector.clases_aceptadas: continue
            
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            
            altura_obj = y2 - y1
            porc = (altura_obj / img_height) * 100.0
            error_altura = abs(porc - 60.0)
            puntaje_altura = 5.0 - (error_altura / 40.0) * 5.0
            
            punto_medio_img = img_width / 2.0
            punto_medio_obj = (x2 + x1) / 2.0
            error_centrado = abs(punto_medio_obj - punto_medio_img)
            max_err_h = img_width / 2.0
            puntaje_centrado = 5.0 - (error_centrado / max_err_h) * 5.0
            
            punto_medio_img_v = img_height / 2.0
            punto_medio_obj_v = (y2 + y1) / 2.0
            error_centrado_v = abs(punto_medio_obj_v - punto_medio_img_v)
            max_err_v = img_height / 2.0
            puntaje_centrado_v = 5.0 - (error_centrado_v / max_err_v) * 5.0
            
            puntaje_act = puntaje_altura + puntaje_centrado + puntaje_centrado_v
            
            margen_h = img_width * 0.05
            margen_v = img_height * 0.05
            margen_pen = 0
            if x1 <= margen_h or x2 >= img_width - margen_h or y1 <= margen_v or y2 >= img_height - margen_v:
                margen_pen = 2.0
                puntaje_act -= 2.0
                
            print(f"Clase: {class_name}, Conf: {conf}")
            print(f"Box: x1={x1:.1f}, y1={y1:.1f}, x2={x2:.1f}, y2={y2:.1f}")
            print(f"Altura obj: {porc:.1f}% -> ptj_alt: {puntaje_altura:.2f}")
            print(f"Centrado H: err={error_centrado:.1f} -> ptj_h: {puntaje_centrado:.2f}")
            print(f"Centrado V: err={error_centrado_v:.1f} -> ptj_v: {puntaje_centrado_v:.2f}")
            print(f"Margen pen: {margen_pen}")
            print(f"Puntaje total: {puntaje_act:.2f}")
