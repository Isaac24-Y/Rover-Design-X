# mapping_system.py - Sistema de Mapeo del Rover
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Wedge
from matplotlib.animation import FuncAnimation
import threading
import time
from collections import deque
import math

class RoverMapper:
    """Sistema de mapeo 2D usando IMU y sensor de distancia"""
    
    def __init__(self, map_size=1000):
        """
        Inicializa el sistema de mapeo
        map_size: tamaño del mapa en centímetros
        """
        self.map_size = map_size
        
        # Posición actual del rover (x, y, theta)
        self.x = map_size / 2  # Centro del mapa
        self.y = map_size / 2
        self.theta = 0  # Ángulo en radianes (0 = Norte)
        
        # Histórico de posiciones
        self.trajectory = deque(maxlen=500)
        self.trajectory.append((self.x, self.y))
        
        # Puntos de obstáculos detectados
        self.obstacles = []
        
        # Datos del IMU
        self.accel_x = 0
        self.accel_y = 0
        self.accel_z = 0
        self.gyro_x = 0
        self.gyro_y = 0
        self.gyro_z = 0
        
        # Sensor de distancia
        self.distance_mm = 0
        
        # Velocidad estimada (cm/s)
        self.velocity = 0
        self.last_update_time = time.time()
        
        # Estado del movimiento
        self.moving_forward = False
        self.moving_backward = False
        self.turning_left = False
        self.turning_right = False
        
        # Configuración de odometría
        self.wheel_base = 20  # Distancia entre ruedas en cm
        self.turn_rate = 0.5  # Radianes por segundo al girar
        
        # Lock para thread-safety
        self.lock = threading.Lock()
        
        # Marcadores detectados
        self.markers = []
        
    def update_imu(self, ax, ay, az, gx, gy, gz):
        """Actualiza datos del IMU"""
        with self.lock:
            self.accel_x = ax
            self.accel_y = ay
            self.accel_z = az
            self.gyro_x = gx
            self.gyro_y = gy
            self.gyro_z = gz
    
    def update_distance(self, distance_mm):
        """Actualiza distancia frontal"""
        with self.lock:
            self.distance_mm = distance_mm
    
    def set_movement_state(self, forward=False, backward=False, left=False, right=False):
        """Actualiza el estado de movimiento del rover"""
        with self.lock:
            self.moving_forward = forward
            self.moving_backward = backward
            self.turning_left = left
            self.turning_right = right
    
    def update_odometry(self):
        """Actualiza la odometría basada en el estado de movimiento y tiempo"""
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        with self.lock:
            # Velocidad lineal estimada (cm/s)
            base_speed = 10  # cm/s cuando se mueve
            
            # Actualizar rotación
            if self.turning_left:
                self.theta += self.turn_rate * dt
            elif self.turning_right:
                self.theta -= self.turn_rate * dt
            
            # Normalizar ángulo entre -π y π
            self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi
            
            # Actualizar posición
            if self.moving_forward:
                self.velocity = base_speed
                dx = self.velocity * math.cos(self.theta) * dt
                dy = self.velocity * math.sin(self.theta) * dt
                self.x += dx
                self.y += dy
            elif self.moving_backward:
                self.velocity = -base_speed
                dx = self.velocity * math.cos(self.theta) * dt
                dy = self.velocity * math.sin(self.theta) * dt
                self.x += dx
                self.y += dy
            else:
                self.velocity = 0
            
            # Agregar a trayectoria
            self.trajectory.append((self.x, self.y))
            
            # Registrar obstáculo si hay detección válida
            if self.distance_mm > 0 and self.distance_mm < 2000:  # Entre 0 y 200cm
                distance_cm = self.distance_mm / 10.0
                obs_x = self.x + distance_cm * math.cos(self.theta)
                obs_y = self.y + distance_cm * math.sin(self.theta)
                
                # Evitar duplicados cercanos
                is_duplicate = False
                for ox, oy in self.obstacles[-50:]:  # Revisar últimos 50
                    if math.sqrt((obs_x - ox)**2 + (obs_y - oy)**2) < 5:  # 5cm
                        is_duplicate = True
                        break
                
                if not is_duplicate:
                    self.obstacles.append((obs_x, obs_y))
    
    def add_marker(self, marker_type, distance_cm=None):
        """Agrega un marcador detectado en la posición actual"""
        with self.lock:
            if distance_cm is None:
                # Marcador en la posición actual
                marker_x = self.x
                marker_y = self.y
            else:
                # Marcador a cierta distancia frontal
                marker_x = self.x + distance_cm * math.cos(self.theta)
                marker_y = self.y + distance_cm * math.sin(self.theta)
            
            self.markers.append({
                'type': marker_type,
                'x': marker_x,
                'y': marker_y,
                'timestamp': time.time()
            })
    
    def get_map_data(self):
        """Retorna todos los datos del mapa de forma thread-safe"""
        with self.lock:
            return {
                'position': (self.x, self.y, self.theta),
                'trajectory': list(self.trajectory),
                'obstacles': self.obstacles.copy(),
                'markers': self.markers.copy(),
                'distance': self.distance_mm,
                'velocity': self.velocity
            }
    
    def reset_map(self):
        """Reinicia el mapa"""
        with self.lock:
            self.x = self.map_size / 2
            self.y = self.map_size / 2
            self.theta = 0
            self.trajectory.clear()
            self.trajectory.append((self.x, self.y))
            self.obstacles.clear()
            self.markers.clear()


class MapVisualizer:
    """Visualización del mapa en tiempo real con Matplotlib"""
    
    def __init__(self, mapper, update_interval=100):
        """
        mapper: instancia de RoverMapper
        update_interval: intervalo de actualización en ms
        """
        self.mapper = mapper
        self.update_interval = update_interval
        
        # Configurar figura
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.fig.canvas.manager.set_window_title('Mapa del Entorno - Rover')
        
        # Elementos gráficos
        self.trajectory_line, = self.ax.plot([], [], 'b-', linewidth=2, label='Trayectoria')
        self.obstacles_scatter = self.ax.scatter([], [], c='red', s=30, marker='x', label='Obstáculos')
        self.rover_body = None
        self.rover_direction = None
        self.distance_line = None
        
        # Marcadores
        self.marker_colors = {
            'Cruz': 'purple',
            'T': 'orange',
            'Circulo': 'green',
            'Cuadrado': 'blue',
            'Triangulo': 'yellow'
        }
        self.marker_artists = []
        
        # Configurar ejes
        self.ax.set_xlim(0, mapper.map_size)
        self.ax.set_ylim(0, mapper.map_size)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('X (cm)', fontsize=12)
        self.ax.set_ylabel('Y (cm)', fontsize=12)
        self.ax.set_title('Mapa del Entorno Explorado', fontsize=14, fontweight='bold')
        self.ax.legend(loc='upper right')
        
        # Texto de información
        self.info_text = self.ax.text(0.02, 0.98, '', transform=self.ax.transAxes,
                                     verticalalignment='top', fontsize=10,
                                     bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        # Animación
        self.anim = None
    
    def update_frame(self, frame):
        """Actualiza el frame de la animación"""
        data = self.mapper.get_map_data()
        
        # Actualizar trayectoria
        if len(data['trajectory']) > 0:
            traj = np.array(data['trajectory'])
            self.trajectory_line.set_data(traj[:, 0], traj[:, 1])
        
        # Actualizar obstáculos
        if len(data['obstacles']) > 0:
            obs = np.array(data['obstacles'])
            self.obstacles_scatter.set_offsets(obs)
        else:
            self.obstacles_scatter.set_offsets(np.empty((0, 2)))
        
        # Actualizar rover
        x, y, theta = data['position']
        
        # Remover rover anterior
        if self.rover_body:
            self.rover_body.remove()
        if self.rover_direction:
            self.rover_direction.remove()
        if self.distance_line:
            self.distance_line.remove()
        
        # Dibujar cuerpo del rover (círculo)
        self.rover_body = Circle((x, y), 5, color='blue', alpha=0.7, zorder=5)
        self.ax.add_patch(self.rover_body)
        
        # Dibujar dirección (flecha)
        arrow_length = 10
        dx = arrow_length * math.cos(theta)
        dy = arrow_length * math.sin(theta)
        self.rover_direction = self.ax.arrow(x, y, dx, dy, head_width=3, 
                                            head_length=4, fc='darkblue', 
                                            ec='darkblue', zorder=6)
        
        # Dibujar línea de distancia
        if data['distance'] > 0 and data['distance'] < 2000:
            dist_cm = data['distance'] / 10.0
            end_x = x + dist_cm * math.cos(theta)
            end_y = y + dist_cm * math.sin(theta)
            self.distance_line, = self.ax.plot([x, end_x], [y, end_y], 
                                               'g--', linewidth=1.5, alpha=0.6)
        
        # Actualizar marcadores
        for artist in self.marker_artists:
            artist.remove()
        self.marker_artists.clear()
        
        for marker in data['markers']:
            color = self.marker_colors.get(marker['type'], 'gray')
            circle = Circle((marker['x'], marker['y']), 8, 
                          color=color, alpha=0.6, zorder=4)
            self.ax.add_patch(circle)
            self.marker_artists.append(circle)
            
            # Etiqueta del marcador
            text = self.ax.text(marker['x'], marker['y'], marker['type'][0], 
                              ha='center', va='center', fontsize=8, 
                              fontweight='bold', color='white', zorder=5)
            self.marker_artists.append(text)
        
        # Actualizar información
        angle_deg = math.degrees(theta)
        info = f"Posición: ({x:.1f}, {y:.1f}) cm\n"
        info += f"Ángulo: {angle_deg:.1f}°\n"
        info += f"Velocidad: {data['velocity']:.1f} cm/s\n"
        info += f"Distancia: {data['distance']:.0f} mm\n"
        info += f"Obstáculos: {len(data['obstacles'])}\n"
        info += f"Marcadores: {len(data['markers'])}"
        self.info_text.set_text(info)
        
        return [self.trajectory_line, self.obstacles_scatter, self.info_text]
    
    def start(self):
        """Inicia la visualización animada"""
        self.anim = FuncAnimation(self.fig, self.update_frame, 
                                 interval=self.update_interval, 
                                 blit=False, cache_frame_data=False)
        plt.show(block=False)
    
    def show(self):
        """Muestra la ventana del mapa (blocking)"""
        plt.show()
    
    def save_map(self, filename='mapa_rover.png'):
        """Guarda el mapa actual como imagen"""
        self.fig.savefig(filename, dpi=300, bbox_inches='tight')
        print(f"Mapa guardado como {filename}")


# Thread de actualización de odometría
class OdometryThread(threading.Thread):
    """Thread que actualiza continuamente la odometría"""
    
    def __init__(self, mapper, update_rate=20):
        """
        mapper: instancia de RoverMapper
        update_rate: actualizaciones por segundo
        """
        super().__init__(daemon=True)
        self.mapper = mapper
        self.update_rate = update_rate
        self.running = True
    
    def run(self):
        """Ejecuta el loop de actualización"""
        while self.running:
            self.mapper.update_odometry()
            time.sleep(1.0 / self.update_rate)
    
    def stop(self):
        """Detiene el thread"""
        self.running = False


# Ejemplo de uso
if __name__ == "__main__":
    # Crear mapeador
    mapper = RoverMapper(map_size=500)
    
    # Iniciar thread de odometría
    odom_thread = OdometryThread(mapper, update_rate=20)
    odom_thread.start()
    
    # Crear visualizador
    visualizer = MapVisualizer(mapper, update_interval=50)
    
    # Simular movimiento
    def simulate_movement():
        """Simula movimiento del rover para prueba"""
        time.sleep(1)
        
        # Avanzar
        mapper.set_movement_state(forward=True)
        time.sleep(3)
        
        # Girar derecha
        mapper.set_movement_state(right=True)
        time.sleep(1)
        
        # Avanzar
        mapper.set_movement_state(forward=True)
        mapper.update_distance(500)  # 50cm de obstáculo
        time.sleep(2)
        
        # Agregar marcador
        mapper.add_marker('Circulo')
        
        # Girar izquierda
        mapper.set_movement_state(left=True)
        time.sleep(1)
        
        # Avanzar
        mapper.set_movement_state(forward=True)
        time.sleep(3)
        
        # Detener
        mapper.set_movement_state()
    
    # Iniciar simulación en thread
    sim_thread = threading.Thread(target=simulate_movement, daemon=True)
    sim_thread.start()
    
    # Mostrar visualización
    visualizer.start()
    visualizer.show()