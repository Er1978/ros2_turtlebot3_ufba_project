# Local: /home/turtlebot3_ws/src/turtlebot3_ufba/turtlebot3_ufba/turtle_fuzzy_node.py
import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy, QoSHistoryPolicy
import math
import time
import tf_transformations
import numpy as np

# --- IMPORTAÇÕES PARA PLOTAGEM ---
import matplotlib.pyplot as plt
# ---------------------------------

class FuzzyController:
    """Classe para um controlador Fuzzy."""
    def __init__(self):
        pass

    def _membership_error(self, error):
        """Calcula a pertinência do erro às funções 'Negativo', 'Zero' e 'Positivo'."""
        # Funções de pertinência triangulares simples
        neg = max(0, min(-error / math.pi, 1))
        zero = max(0, 1 - abs(error) / (math.pi / 4))
        pos = max(0, min(error / math.pi, 1))
        return neg, zero, pos

    def _defuzzify(self, left, zero, right):
        """Defuzzifica a saída para obter uma velocidade angular nítida."""
        # Defuzzificação ponderada simples
        # Saídas nítidas para cada regra
        
        # =======================================================================
        # CORREÇÃO APLICADA AQUI
        # Invertemos os sinais para corresponder ao sistema de coordenadas do ROS.
        # =======================================================================
        turn_left_speed = 1.5   # Girar para a esquerda (anti-horário) é POSITIVO em ROS
        no_turn_speed = 0.0
        turn_right_speed = -1.5  # Girar para a direita (horário) é NEGATIVO em ROS
        
        numerator = (left * turn_left_speed) + (zero * no_turn_speed) + (right * turn_right_speed)
        denominator = left + zero + right
        
        if denominator == 0:
            return 0.0
        return numerator / denominator

    def update(self, error):
        """Calcula a velocidade angular com base no erro de orientação."""
        neg, zero, pos = self._membership_error(error)

        # Regras Fuzzy
        # 1. Se erro é Negativo, então velocidade é Girar Esquerda (velocidade positiva)
        # 2. Se erro é Zero, então velocidade é Não Girar (velocidade zero)
        # 3. Se erro é Positivo, então velocidade é Girar Direita (velocidade negativa)
        
        # A "força" de cada regra é simplesmente o valor de pertinência da entrada
        rule1_strength = neg
        rule2_strength = zero
        rule3_strength = pos

        # A saída é a defuzzificação das regras ativadas
        return self._defuzzify(rule1_strength, rule2_strength, rule3_strength)


class TurtleFuzzyController(Node):
    def __init__(self):
        super().__init__('turtle_fuzzy_controller_node')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile)
        
        self.current_position = None
        self.current_angle=None
        self.is_moving = False
        self.is_turning=False
        self.get_logger().info('Nó de Controle Fuzzy iniciado!')

        self.plot_x_real = []
        self.plot_y_real = []
        self.plot_teta_real = []
        
        self.plot_sample_rate = 5
        self.odom_msg_count = 0

    def odom_callback(self,msg: Odometry):
        self.current_position = msg.pose.pose.position
        orientation_object=msg.pose.pose.orientation
        orientation_q= tf_transformations.euler_from_quaternion([orientation_object.x,orientation_object.y,orientation_object.z,orientation_object.w])
        self.current_angle= orientation_q[2]
        
        self.odom_msg_count += 1
        if self.odom_msg_count % self.plot_sample_rate == 0:
            self.plot_x_real.append(self.current_position.x)
            self.plot_y_real.append(self.current_position.y)
            self.plot_teta_real.append(self.current_angle)

    def normalize_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi     

    def follow_track(self, track_line, controller_type):
        while self.current_position is None:
            self.get_logger().info('Aguardando a primeira leitura de odometria...')
            rclpy.spin_once(self)
            time.sleep(0.1)
        
        n=0
        for target_position in track_line:
            angle=target_position[0] 
            distance=target_position[1]
            n=n+1
            self.get_logger().info(f'Percorrendo segmento {str(n)}. Rotação: {angle} º Distância: {distance}m')

            if(controller_type=='F'):
                self.turn_to_angle_fuzzy(angle)
                self.move_robot(distance,0.2)
            else:
                self.turn_to_angle(angle,0.3)
                self.move_robot(distance,0.2)

        self.get_logger().info('Caminho finalizado')

    def turn_to_angle_fuzzy(self, target_angle_degrees, tolerance_degrees=0.5): # Aumentei a tolerância um pouco
        if self.is_turning:
            self.get_logger().info('Robo sendo redirecionado')
            return

        self.is_turning = True
        
        initial_angle_at_start_of_turn = self.current_angle
        relative_angle_rad = math.radians(target_angle_degrees)
        target_absolute_angle_rad = self.normalize_angle(initial_angle_at_start_of_turn + relative_angle_rad)
        
        tolerance_rad = math.radians(tolerance_degrees)
        
        fuzzy_controller = FuzzyController()
        
        self.get_logger().info(f'Iniciando rotação para {target_angle_degrees}° (Alvo em radianos: {target_absolute_angle_rad:.3f})')
        
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            # Esta linha está correta, não a altere.
            # Um erro negativo significa que precisamos girar para a esquerda (velocidade positiva)
            error = self.normalize_angle(self.current_angle - target_absolute_angle_rad)

            if abs(error) <= tolerance_rad:
                self.get_logger().info('Alvo de rotação atingido com sucesso.')
                break

            angular_speed = fuzzy_controller.update(error)
            angular_speed = max(min(angular_speed, 2.0), -2.0) # Limita a velocidade máxima

            twist_msg = Twist()
            twist_msg.angular.z = angular_speed
            self.publisher_.publish(twist_msg)

            # Para o log, mostramos o erro de forma mais intuitiva (alvo - atual)
            display_error = self.normalize_angle(target_absolute_angle_rad - self.current_angle)
            self.get_logger().info(
                f'Angulo atual: {math.degrees(self.current_angle):.2f}° | Erro: {math.degrees(display_error):.2f}° | Vel. Aplicada: {angular_speed:.3f} rad/s', 
                throttle_duration_sec=0.5)

        self.stop_robot()
        self.is_turning = False

    def turn_to_angle(self,target_angle_degrees, angle_speed=0.1):
        angle_speed = max(min(angle_speed, 2.0), -2.0)
        target_angle_rad=math.radians(target_angle_degrees+self.current_angle)
        rotated_angle=0.00
        rotated_previous=self.current_angle
        self.get_logger().info(f'Angulo atual: {math.degrees(rotated_previous)}')
        if self.is_turning:
            self.get_logger().info('Robo sendo redirecionado')
            return
        self.is_turning=True
        twist_msg = Twist()
        if(target_angle_degrees<0):
            angle_speed=angle_speed * -1
        twist_msg.angular.z=angle_speed
        twist_msg.linear.x = 0.0
        while abs(rotated_angle) < abs(target_angle_rad):
            self.publisher_.publish(twist_msg)
            rclpy.spin_once(self)
            if self.current_angle:
                rotated_angle = self.normalize_angle(self.current_angle - rotated_previous)
                rotated_angle_degrees= math.degrees(rotated_angle)
                self.get_logger().info(f'Robô girado em {rotated_angle_degrees:.2f}º', throttle_duration_sec=0.5)
        self.stop_robot()
        self.get_logger().info('Robô direcionado')

    def move_robot(self, target_distance, linear_speed=0.2):
        linear_speed=min(linear_speed,0.2)
        if self.is_moving:
            self.get_logger().warn('Robô em deslocamento.')
            return
        self.is_moving = True
        start_position = self.current_position
        distance_traveled = 0.0
        self.get_logger().info(f'Iniciando movimento. Distância a percorrer: {target_distance:.2f}m a {linear_speed:.2f}m/s.')
        twist_msg = Twist()
        twist_msg.linear.x = linear_speed
        while distance_traveled < target_distance:
            self.publisher_.publish(twist_msg)
            rclpy.spin_once(self)
            if self.current_position:
                distance_traveled = math.sqrt(
                    (self.current_position.x - start_position.x)**2 +
                    (self.current_position.y - start_position.y)**2
                )
            self.get_logger().info(f'Distância percorrida: {distance_traveled:.2f}m', throttle_duration_sec=1)
        self.stop_robot()
        self.get_logger().info(f'Distância de {target_distance:.2f}m atingida.')
        self.is_moving = False

    def stop_robot(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.publisher_.publish(stop_msg)
        self.is_moving = False        
        self.is_turning=False
        
    def plot_trajectory_comparison(self):
        self.get_logger().info("Gerando gráfico de trajetória...")
        plt.figure(figsize=(12, 10))
        plt.plot(self.plot_x_real, self.plot_y_real, color='blue', linestyle='-', linewidth=2, label='Trajeto Real (Fuzzy)')
        arrow_skip = 10 
        arrow_length = 0.1 
        for i in range(0, len(self.plot_x_real), arrow_skip):
            dx_arrow = arrow_length * np.cos(self.plot_teta_real[i])
            dy_arrow = arrow_length * np.sin(self.plot_teta_real[i])
            plt.arrow(self.plot_x_real[i], self.plot_y_real[i], dx_arrow, dy_arrow, 
                      head_width=0.02, head_length=0.03, fc='darkblue', ec='darkblue', 
                      length_includes_head=True, alpha=0.7)
        plt.xlabel('Posição X (m)')
        plt.ylabel('Posição Y (m)')
        plt.title('Trajetória: Real (Fuzzy)')
        plt.grid(True)
        plt.axis('equal') 
        plt.legend()
        plt.savefig('/home/user/trajeto_gazebo_fuzzy.png') 
        plt.show()
        self.get_logger().info("Gráfico de trajetória salvo em /home/user/trajeto_gazebo_fuzzy.png")

def main(args=None):
    rclpy.init(args=args)        
    parser = argparse.ArgumentParser(description='Controla o TurtleBot3 com Lógica Fuzzy')
    
    parser.add_argument(
        '--control', 
        type=str, 
        default='F', 
        choices=['N', 'F'],
        help="Define se o controle utilizará Fuzzy (F) ou não (N 'opcional')."
    )

    known_args, _ = parser.parse_known_args()

    trilha = [
        [  0, 2.0], [ 45, 1.0], [ -90, 1.0], [ 45, 1.0], [ 90, 3.0], [ 90, 2.0],
        [ 45, 2.0], [ -45, 1.0], [ -90, 1.5], [ -90, 1.0], [ 45, 2.0], [ -45, 2.0]
    ]
    
    turtle_controller = TurtleFuzzyController()
    time.sleep(6.0) 
    option=known_args.control
    if(option=='F'):
        turtle_controller.get_logger().info('Controle Fuzzy ligado')
    else:
        turtle_controller.get_logger().info('Controle Simples')
                  
    turtle_controller.follow_track(trilha, option)
    turtle_controller.plot_trajectory_comparison()
    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()