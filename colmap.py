import os
import cv2
import shutil
import traceback
import subprocess
import numpy as np


class Colmap_Control:
    def __init__(self, camera, colmap_exec="colmap", enable_dense=True):
        self.camera = camera
        self.colmap_exec = colmap_exec
        self.enable_dense = enable_dense
        self.base_dir = os.path.dirname(os.path.abspath(__file__))
        self.images_dir = os.path.join(self.base_dir, "fotos_capturadas")
        self.output_dir = os.path.join(self.base_dir, "colmap_output")
        self.logs_dir = os.path.join(self.output_dir, "logs")
        self.sparse_dir = os.path.join(self.output_dir, "sparse")
        self.dense_dir = os.path.join(self.output_dir, "dense")
        self.database_path=os.path.abspath(os.path.join(self.output_dir,"database.db"))
        self.images_dir=os.path.abspath(self.images_dir)
        self._create_directories()
        #print("COLMAP EXISTS:",os.path.exists(self.colmap_exec))

    def _create_directories(self):
        os.makedirs(self.images_dir, exist_ok=True)
        os.makedirs(self.output_dir, exist_ok=True)
        os.makedirs(self.logs_dir, exist_ok=True)
        os.makedirs(self.sparse_dir, exist_ok=True)

    def _write_log(self,filename,text):
        os.makedirs(self.logs_dir,exist_ok=True)

        with open(
            os.path.join(self.logs_dir,filename),
            "w",
            encoding="utf-8"
        ) as f:
            f.write(text)

    def _run(self,args,logfile):
        env_limpio = os.environ.copy()
        
        # 1. Encontrar y borrar TODA variable contaminada por OpenCV
        claves_a_borrar = [k for k in env_limpio.keys() if "QT" in k]
        for k in claves_a_borrar:
            del env_limpio[k]
            
        # 2. Forzar explícitamente a COLMAP a no usar interfaz gráfica
        env_limpio["QT_QPA_PLATFORM"] = "offscreen"

        try:
            print("EJECUTANDO:")
            print(args)

            process=subprocess.run(
                args,
                capture_output=True,
                text=True,
                env=env_limpio
            )

            print("RETURN CODE:",process.returncode)

            text=""

            if process.stdout:
                text+=process.stdout+"\n"

            if process.stderr:
                text+=process.stderr+"\n"

            self._write_log(logfile,text)

            return process.returncode==0

        except Exception:
            self._write_log(logfile,traceback.format_exc())
            print(traceback.format_exc())
            return False

    def count_images(self):
        if not os.path.exists(self.images_dir):
            return 0
        images = [f for f in os.listdir(self.images_dir) if f.lower().endswith((".jpg", ".jpeg"))]
        return len(images)

    def save_image(self):
        try:
            image = self.camera.getImage()
            if image is None:
                return False

            width = self.camera.getWidth()
            height = self.camera.getHeight()

            img = np.frombuffer(image,dtype=np.uint8).reshape((height,width,4))
            img = cv2.cvtColor(img,cv2.COLOR_BGRA2BGR)

            idx = self.count_images()
            filename = os.path.join(self.images_dir,f"foto_{idx:04d}.jpg")

            cv2.imwrite(filename,img,[cv2.IMWRITE_JPEG_QUALITY,95])

            saved=cv2.imread(filename)

            return True

        except Exception:
            self._write_log("save_image_error.txt",traceback.format_exc())
            return False

    def clean_output(self):
        try:
            os.makedirs(self.output_dir,exist_ok=True)
            os.makedirs(self.logs_dir,exist_ok=True)
            os.makedirs(self.sparse_dir,exist_ok=True)
            os.makedirs(self.dense_dir,exist_ok=True)

            if os.path.exists(self.database_path):
                try:
                    os.remove(self.database_path)
                except:
                    pass

            if os.path.exists(self.sparse_dir):
                shutil.rmtree(self.sparse_dir,ignore_errors=True)
            os.makedirs(self.sparse_dir,exist_ok=True)

            if os.path.exists(self.dense_dir):
                shutil.rmtree(self.dense_dir,ignore_errors=True)
            os.makedirs(self.dense_dir,exist_ok=True)

        except Exception:
            os.makedirs(self.logs_dir,exist_ok=True)
            with open(
                os.path.join(self.logs_dir,"clean_error.txt"),
                "w",
                encoding="utf-8"
            ) as f:
                f.write(traceback.format_exc())

    def sparse_exists(self):
        if not os.path.exists(self.sparse_dir):
            return False
        subdirs = [d for d in os.listdir(self.sparse_dir) if os.path.isdir(os.path.join(self.sparse_dir, d))]
        if len(subdirs) == 0:
            return False
        model = os.path.join(self.sparse_dir, subdirs[0])
        required = ["cameras.bin", "images.bin", "points3D.bin"]
        for r in required:
            if not os.path.exists(os.path.join(model, r)):
                return False
        return True

    def run_colmap(self):
        try:
            os.makedirs(self.output_dir,exist_ok=True)
            os.makedirs(self.logs_dir,exist_ok=True)
            os.makedirs(self.sparse_dir,exist_ok=True)
            os.makedirs(self.dense_dir,exist_ok=True)

            print("DATABASE:",self.database_path)
            print("IMAGES:",self.images_dir)
            print("IMAGES EXISTS:",os.path.exists(self.images_dir))
            print("OUTPUT EXISTS:",os.path.exists(self.output_dir))
            print("NUM IMAGES:",self.count_images())
            n = self.count_images()
            if n < 40:
                self._write_log("error.txt", f"Solo hay {n} imágenes.")
                return False
            self.clean_output()
            print("COLMAP:",self.colmap_exec)
            print("DATABASE EXISTS:",os.path.exists(self.database_path))
            print("DATABASE DIR:",os.path.dirname(self.database_path))
            print("DATABASE DIR EXISTS:",os.path.exists(os.path.dirname(self.database_path)))
            feature_cmd = [
                self.colmap_exec, "feature_extractor",
                "--database_path", self.database_path,
                "--image_path", self.images_dir,
                "--ImageReader.camera_model", "PINHOLE",
                "--ImageReader.single_camera", "1",
                "--SiftExtraction.max_num_features", "16384",
                #"--SiftExtraction.max_image_size", "2400"
            ]
            if not self._run(feature_cmd, "feature_extractor.log"):
                return False
            print("DB CREATED:",os.path.exists(self.database_path))
            match_cmd=[
                self.colmap_exec,
                "sequential_matcher",
                "--database_path",
                self.database_path
            ]
            if not self._run(match_cmd, "matcher.log"):
                print("MATCHER FALLÓ")
                return False

            print("MATCHER OK")
            print("ENTRANDO A MAPPER")
            # if not self._run(match_cmd, "matcher.log"):
            #     return False
            mapper_cmd=[
                self.colmap_exec,
                "mapper",
                "--database_path",self.database_path,
                "--image_path",self.images_dir,
                "--output_path",self.sparse_dir,
                "--Mapper.init_min_num_inliers","4",
                "--Mapper.abs_pose_min_num_inliers","4",
                "--Mapper.min_num_matches","8",
                "--Mapper.ba_refine_focal_length","0",
                "--Mapper.ba_refine_principal_point","0",
                "--Mapper.ba_refine_extra_params","0"
            ]
            print("LANZANDO MAPPER")

            if not self._run(mapper_cmd, "mapper.log"):
                print("MAPPER FALLÓ")
                return False

            print("MAPPER TERMINÓ")
            
            print("SPARSE CONTENT:")
            print(os.listdir(self.sparse_dir))

            for root, dirs, files in os.walk(self.sparse_dir):
                print(root)
                print(files)

            print("MAPPER OK")
            if not self._run(mapper_cmd, "mapper.log"):
                return False
            if not self.sparse_exists():
                self._write_log("error.txt", "No se generó sparse.")
                return False
            if self.enable_dense:
                subdirs = [d for d in os.listdir(self.sparse_dir) if os.path.isdir(os.path.join(self.sparse_dir, d))]
                model = os.path.join(self.sparse_dir, subdirs[0])
                if os.path.exists(self.dense_dir):
                    shutil.rmtree(self.dense_dir)
                os.makedirs(self.dense_dir, exist_ok=True)
                undist_cmd = [
                    "xvfb-run", "-a",
                    self.colmap_exec, "image_undistorter",
                    "--image_path", self.images_dir,
                    "--input_path", model,
                    "--output_path", self.dense_dir,
                    "--output_type", "COLMAP"
                ]
                self._run(undist_cmd, "undistorter.log")
                patch_cmd = [
                    "xvfb-run", "-a",
                    self.colmap_exec, "patch_match_stereo",
                    "--workspace_path", self.dense_dir,
                    "--workspace_format", "COLMAP",
                    "--PatchMatchStereo.geom_consistency", "true"
                ]
                self._run(patch_cmd, "patch_match.log")
                fused = os.path.join(self.dense_dir, "fused.ply")
                fusion_cmd = [
                    "xvfb-run", "-a",
                    self.colmap_exec, "stereo_fusion",
                    "--workspace_path", self.dense_dir,
                    "--workspace_format", "COLMAP",
                    "--input_type", "geometric",
                    "--output_path", fused
                ]
                self._run(fusion_cmd, "fusion.log")
            print("COLMAP TERMINADO")
            return True
        except Exception:
            self._write_log("exception.txt", traceback.format_exc())
            return False
        
    def reset_project(self):
        try:
            if os.path.exists(self.images_dir):
                shutil.rmtree(self.images_dir)

            if os.path.exists(self.output_dir):
                shutil.rmtree(self.output_dir)

            os.makedirs(self.images_dir,exist_ok=True)
            os.makedirs(self.output_dir,exist_ok=True)
            os.makedirs(self.logs_dir,exist_ok=True)
            os.makedirs(self.sparse_dir,exist_ok=True)
            os.makedirs(self.dense_dir,exist_ok=True)

        except Exception:
            print(traceback.format_exc())