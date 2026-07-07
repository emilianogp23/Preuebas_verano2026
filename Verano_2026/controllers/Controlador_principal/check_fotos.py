import os
import cv2
from yolo_detector import YoloDetector

detector = YoloDetector()

if not os.path.exists(detector.ruta_fotos):
    print("No fotos directory")
    exit()

fotos = os.listdir(detector.ruta_fotos)
if len(fotos) == 0:
    print("No fotos found")
    exit()

for f in fotos:
    print(f"\n--- {f} ---")
    img_path = os.path.join(detector.ruta_fotos, f)
    img_np = cv2.imread(img_path)
    if img_np is None: continue
    
    img_height, img_width = img_np.shape[:2]
    
    results = detector.model(img_np, verbose=False)
    boxes_found = 0
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
            margen = False
            if x1 <= margen_h or x2 >= img_width - margen_h or y1 <= margen_v or y2 >= img_height - margen_v:
                margen = True
                puntaje_act -= 2.0
                
            print(f"[{class_name}] conf={conf} Altura={porc:.1f}%(ptj={puntaje_altura:.1f}) CtrH={error_centrado:.1f}(ptj={puntaje_centrado:.1f}) CtrV={error_centrado_v:.1f}(ptj={puntaje_centrado_v:.1f}) Margen={margen} TOTAL={puntaje_act:.1f}")
            boxes_found += 1
            
    if boxes_found == 0:
        print("NO OBJECTS DETECTED")
