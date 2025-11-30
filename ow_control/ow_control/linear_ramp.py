import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import Imu
import math
import time

class LinearRampNode(Node):
    def __init__(self):
        super().__init__('linear_ramp_node')
        
        # Parámetros (puedes hacerlos configurables después)
        self.ramp_time = 1.0  # Segundos para llegar a la meta
        self.frequency = 20.0 # Hz
        self.step_period = 1.0 / self.frequency
        self.use_foc = True   # Field Oriented Control (Headless Mode)
        
        # Estado interno (Velocidades en el frame del MUNDO/JOYSTICK)
        self.current_world_vel = Twist()
        self.target_world_vel = Twist()
        
        self.inc_x = 0.0
        self.inc_y = 0.0
        self.inc_theta = 0.0
        self.last_time = self.get_clock().now()
        self.robot_yaw = 0.0
        
        # Suscriptor (Escucha al teclado/joystick)
        self.sub = self.create_subscription(
            TwistStamped,
            'cmd_vel_input', # Entrada
            self.cmd_vel_callback,
            10)

        # Suscriptor de IMU (Para saber la orientación del robot)
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu', 
            self.imu_callback,
            10)
            
        # Publicador (Habla al controlador)
        self.pub = self.create_publisher(
            TwistStamped,
            'cmd_vel_out', # Salida
            10)
            
        # Timer del bucle de control
        self.timer = self.create_timer(self.step_period, self.control_loop)

    def imu_callback(self, msg):
        # Extraer orientación (Quaternion) -> Yaw (Euler)
        q = msg.orientation
        # Conversión rápida de Quaternion a Yaw
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def cmd_vel_callback(self, msg):
        # Guardamos la velocidad deseada en el frame del MUNDO (tal cual viene del joystick)
        self.target_world_vel = msg.twist
        
        # 1. Calculamos cuántos pasos daremos en total
        total_steps = self.frequency * self.ramp_time
        
        # 2. Calculamos cuánto hay que sumar en X, Y y Theta en cada paso (en frame MUNDO)
        self.inc_x = (self.target_world_vel.linear.x - self.current_world_vel.linear.x) / total_steps
        self.inc_y = (self.target_world_vel.linear.y - self.current_world_vel.linear.y) / total_steps
        self.inc_theta = (self.target_world_vel.angular.z - self.current_world_vel.angular.z) / total_steps
        
    def get_next_velocity(self, current, target, increment):
        distancia = target - current
        if abs(distancia) > abs(increment):
            return current + increment
        else:
            return target

    def control_loop(self):
        # 1. Rampa: Actualizamos la velocidad actual en el frame del MUNDO
        self.current_world_vel.linear.x = self.get_next_velocity(self.current_world_vel.linear.x, self.target_world_vel.linear.x, self.inc_x)
        self.current_world_vel.linear.y = self.get_next_velocity(self.current_world_vel.linear.y, self.target_world_vel.linear.y, self.inc_y)
        self.current_world_vel.angular.z = self.get_next_velocity(self.current_world_vel.angular.z, self.target_world_vel.angular.z, self.inc_theta)

        # 2. Transformación FOC: Mundo -> Robot
        # Ahora que tenemos la velocidad suavizada en el mundo, la rotamos al frame del robot
        final_vel = Twist()
        
        if self.use_foc:
            # v_robot = R(-theta) * v_world
            cos_theta = math.cos(self.robot_yaw)
            sin_theta = math.sin(self.robot_yaw)
            
            vx_world = self.current_world_vel.linear.x
            vy_world = self.current_world_vel.linear.y
            
            final_vel.linear.x = vx_world * cos_theta + vy_world * sin_theta
            final_vel.linear.y = -vx_world * sin_theta + vy_world * cos_theta
            final_vel.angular.z = self.current_world_vel.angular.z
        else:
            final_vel = self.current_world_vel

        # 3. Publicar
        msg_stamped = TwistStamped()
        msg_stamped.header.stamp = self.get_clock().now().to_msg()
        msg_stamped.header.frame_id = 'base_link'
        msg_stamped.twist = final_vel
        self.pub.publish(msg_stamped)

def main(args=None):
    rclpy.init(args=args)
    node = LinearRampNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
