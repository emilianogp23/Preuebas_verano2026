import numpy as np
import math

class OdometriaIMU:
    def __init__(self, init_pos=[0.0, 0.0, 0.0], init_vel=[0.0, 0.0, 0.0]):
        self.position = np.array(init_pos, dtype=float)
        self.velocity = np.array(init_vel, dtype=float)
        self.gravity = np.array([0.0, 0.0, 9.81]) # Asumiendo que el acelerómetro mide la gravedad positiva en Z cuando está en reposo

    def get_rotation_matrix(self, roll, pitch, yaw):
        # Matrices de rotación
        Rx = np.array([
            [1, 0, 0],
            [0, math.cos(roll), -math.sin(roll)],
            [0, math.sin(roll), math.cos(roll)]
        ])
        
        Ry = np.array([
            [math.cos(pitch), 0, math.sin(pitch)],
            [0, 1, 0],
            [-math.sin(pitch), 0, math.cos(pitch)]
        ])
        
        Rz = np.array([
            [math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw), math.cos(yaw), 0],
            [0, 0, 1]
        ])
        
        # Matriz de rotación completa de Body a World (XYZ: Rz * Ry * Rx)
        return Rz @ Ry @ Rx

    def update(self, dt, accel_body, rpy):
        """
        Actualiza la estimación de posición y velocidad.
        dt: delta de tiempo en segundos
        accel_body: lista o array con [ax, ay, az] medido por el acelerómetro (en el frame del dron)
        rpy: lista o array con [roll, pitch, yaw] medido por la unidad inercial (en radianes)
        """
        accel_body = np.array(accel_body)
        
        # 1. Obtener matriz de rotación basada en la orientación actual
        R = self.get_rotation_matrix(rpy[0], rpy[1], rpy[2])
        
        # 2. Rotar la aceleración del frame del cuerpo al frame del mundo
        accel_world = R @ accel_body
        
        # 3. Restar la gravedad para obtener la aceleración lineal real
        linear_accel_world = accel_world - self.gravity
        
        # 4. Integrar para obtener la velocidad (V = V0 + a*dt)
        self.velocity += linear_accel_world * dt
        
        # 5. Integrar para obtener la posición (P = P0 + V*dt)
        self.position += self.velocity * dt
        
        return self.position, self.velocity

