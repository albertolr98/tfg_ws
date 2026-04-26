import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
import time

class LinearRampNode(Node):
    def __init__(self):
        super().__init__('linear_ramp_node')
        
        # Parámetros (puedes hacerlos configurables después)
        self.ramp_time = 0.5  # Segundos para llegar a la meta (igual que kRampTimeSeconds en tmc_driver)
        self.frequency = 20.0 # Hz
        self.step_period = 1.0 / self.frequency
        
        # Estado interno
        self.current_vel = Twist()
        self.target_vel = Twist()
        self.inc_x = 0.0
        self.inc_y = 0.0
        self.inc_theta = 0.0
        self.last_time = self.get_clock().now()
        
        # Suscriptor (Escucha al teclado/joystick)
        self.sub = self.create_subscription(
            TwistStamped,
            'cmd_vel_input', # Entrada
            self.cmd_vel_callback,
            10)
            
        # Publicador (Habla al controlador)
        self.pub = self.create_publisher(
            TwistStamped,
            'cmd_vel_out', # Salida
            10)
            
        # Timer del bucle de control
        self.timer = self.create_timer(self.step_period, self.control_loop)

    def cmd_vel_callback(self, msg):
        self.target_vel = msg.twist
        
        # 1. Calculamos cuántos pasos daremos en total (ej: 20 pasos en 1 segundo)
        total_steps = self.frequency * self.ramp_time
        
        # 2. Calculamos cuánto hay que sumar en X, Y y Theta en cada paso
        self.inc_x = (self.target_vel.linear.x - self.current_vel.linear.x) / total_steps
        self.inc_y = (self.target_vel.linear.y - self.current_vel.linear.y) / total_steps
        self.inc_theta = (self.target_vel.angular.z - self.current_vel.angular.z) / total_steps
        
    def get_next_velocity(self, current, target, increment):
        distancia = target - current
        if abs(distancia) > abs(increment):
            return current + increment
        else:
            return target

    def control_loop(self):
        # 1. Sumar/Restar ese paso a self.current_vel
        self.current_vel.linear.x = self.get_next_velocity(self.current_vel.linear.x, self.target_vel.linear.x, self.inc_x)
        self.current_vel.linear.y = self.get_next_velocity(self.current_vel.linear.y, self.target_vel.linear.y, self.inc_y)
        self.current_vel.angular.z = self.get_next_velocity(self.current_vel.angular.z, self.target_vel.angular.z, self.inc_theta)

        # Preparar el mensaje sellado
        msg_stamped = TwistStamped()
        
        # Rellenamos la cabecera
        msg_stamped.header.stamp = self.get_clock().now().to_msg()
        msg_stamped.header.frame_id = 'base_link' # O el frame que use tu robot
        
        # Metemos tu velocidad calculada dentro del paquete
        msg_stamped.twist = self.current_vel 
    
        # Publicamos el paquete completo
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
