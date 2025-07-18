# Local: /home/turtlebot3_ws/src/turtlebot3_ufba/turtlebot3_ufba/turtle_lqr_node.py
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
from scipy.linalg import solve_continuous_are

# --- IMPORTAÇÕES PARA PLOTAGEM ---
import matplotlib.pyplot as plt
# ---------------------------------

class LQRController:
    """Classe para um controlador LQR."""
    def __init__(self, Q, R):
        self.Q = Q
        self.R = R
        # Simplified state-space model for the robot's orientation
        # x_dot = Ax + Bu
        # For a simple orientation model, A can be considered 0 as the state doesn't change on its own
        # B is 1 as the control input directly affects the angular velocity
        self.A = np.array([[0]])
        self.B = np.array([[1]])
        # Solve the continuous algebraic Riccati equation
        P = solve_continuous_are(self.A, self.B, self.Q, self.R)
        # Calculate the LQR gain
        self.K = np.linalg.inv(self.R) @ self.B.T @ P

    def update(self, error):
        # For a single state system, the control law is u = -Kx
        return -self.K[0, 0] * error

class TurtleLQRController(Node):
    def __init__(self):
        super().__init__('turtle_lqr_controller_node')

        # Define um perfil de Qualidade de Serviço (QoS) confiável para o subscriber de odometria
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Cria um publisher  para os dados de velocidade
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Cria o subscriber para receber dados de odometria
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile)
        
        self.current_position = None
        self.current_angle=None
        self.is_moving = False
        self.is_turning=False
        self.get_logger().info('Nó de Controle LQR iniciado!')

        # --- VARIÁVEIS PARA PLOTAGEM ---
        self.plot_x_real = []
        self.plot_y_real = []
        self.plot_teta_real = []
        
        self.plot_sample_rate = 5
        self.odom_msg_count = 0
        # ---------------------------------

    def odom_callback(self,msg: Odometry):
        """Obtem os dados de odometria"""
        self.current_position = msg.pose.pose.position
        orientation_object=msg.pose.pose.orientation
        orientation_q= tf_transformations.euler_from_quaternion([orientation_object.x,orientation_object.y,orientation_object.z,orientation_object.w])
        self.current_angle= orientation_q[2]
        
        # --- COLETA DE DADOS REAIS PARA PLOTAGEM ---
        self.odom_msg_count += 1
        if self.odom_msg_count % self.plot_sample_rate == 0:
            self.plot_x_real.append(self.current_position.x)
            self.plot_y_real.append(self.current_position.y)
            self.plot_teta_real.append(self.current_angle)
        # -----------------------------------------

    def normalize_angle(self, angle):
        """Normaliza um ângulo para o intervalo [-pi, pi]."""
        return (angle + math.pi) % (2 * math.pi) - math.pi     

    def follow_track(self, track_line, controller_type):
        """Percorre a trilha com as curvas dadas no vetor track_line"""
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

            if(controller_type=='L'):
                self.turn_to_angle_lqr(angle)
                self.move_robot(distance,0.2)
            else:
                self.turn_to_angle(angle,0.3)
                self.move_robot(distance,0.2)

        self.get_logger().info('Caminho finalizado')

    # ===================================================================================
    # FUNÇÃO CORRIGIDA
    # ===================================================================================
    def turn_to_angle_lqr(self, target_angle_degrees, tolerance_degrees=0.05):
        """
        Gira o robô para um ângulo alvo (em graus) usando o controlador LQR.
        """
        if self.is_turning:
            self.get_logger().info('Robo sendo redirecionado')
            return

        self.is_turning = True
        
        initial_angle_at_start_of_turn = self.current_angle
        relative_angle_rad = math.radians(target_angle_degrees)
        target_absolute_angle_rad = self.normalize_angle(initial_angle_at_start_of_turn + relative_angle_rad)
        
        tolerance_rad = math.radians(tolerance_degrees)
        
        # LQR tuning matrices
        Q = np.array([[10.0]])  # State cost
        R = np.array([[1.0]])   # Control cost
        lqr = LQRController(Q, R)
        
        self.get_logger().info(f'Iniciando rotação para {target_angle_degrees}° (Alvo em radianos: {target_absolute_angle_rad:.3f})')
        
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            # --- LINHA CORRIGIDA ---
            # O erro agora é calculado como (atual - alvo) para que a lei de controle -K*erro
            # produza o sinal de velocidade angular correto.
            error = self.normalize_angle(self.current_angle - target_absolute_angle_rad)

            if abs(error) <= tolerance_rad:
                self.get_logger().info('Alvo de rotação atingido com sucesso.')
                break

            # A lei de controle LQR é u = -K*x, onde x é o erro.
            # Com a correção, se o robô precisa girar no sentido anti-horário (positivo),
            # o erro será negativo, resultando em uma velocidade angular positiva.
            angular_speed = lqr.update(error)
            angular_speed = max(min(angular_speed, 2.0), -2.0)

            twist_msg = Twist()
            twist_msg.angular.z = angular_speed
            self.publisher_.publish(twist_msg)

            # O log do erro aqui pode parecer "invertido", mas o que importa é o comportamento do robô.
            # Para evitar confusão, vamos logar o erro como a diferença para o alvo.
            display_error = self.normalize_angle(target_absolute_angle_rad - self.current_angle)
            self.get_logger().info(
                f'Angulo atual: {math.degrees(self.current_angle):.2f}° | Erro: {math.degrees(display_error):.2f}° | Vel. Aplicada: {angular_speed:.3f} rad/s', 
                throttle_duration_sec=0.5)

        self.stop_robot()
        self.is_turning = False
    # ===================================================================================

    def turn_to_angle(self,target_angle_degrees, angle_speed=0.1):
        """Gira o robo para o angulo desejado em graus absolutos(YAW) sem usar LQR"""
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
        
        linear_speed = min(linear_speed, 0.2)
        if self.is_moving:
            self.get_logger().warn('Robô já está em deslocamento.')
            return

        self.is_moving = True
    
    # --- NOVO: Captura da posição e orientação iniciais ---
        start_position = self.current_position
        initial_angle = self.current_angle  # O ângulo que queremos manter
        distance_traveled = 0.0

        # --- NOVO: Instanciação do controlador LQR para orientação ---
        # Usamos os mesmos ganhos do controle de rotação, que são bons para essa tarefa
        Q = np.array([[10.0]])  # Custo do erro de estado (quão importante é ter erro zero)
        R = np.array([[1.0]])   # Custo do esforço de controle (quão "caro" é usar o motor)
        orientation_lqr = LQRController(Q, R)
    
        self.get_logger().info(f'Iniciando movimento com LQR. Distância: {target_distance:.2f}m, Mantendo ângulo: {math.degrees(initial_angle):.2f}°')
    
        twist_msg = Twist()
    
        while distance_traveled < target_distance:
            rclpy.spin_once(self) # Atualiza a odometria (self.current_position e self.current_angle)
        
            # --- LÓGICA DE CONTROLE LQR ---
            #    1. Calcular o erro de orientação
            # Usamos (atual - alvo) para ser compatível com a lei de controle u = -Kx
            angle_error = self.normalize_angle(self.current_angle - initial_angle)
        
            # 2. Obter a velocidade angular corretiva do LQR
            angular_speed_correction = orientation_lqr.update(angle_error)
        
            # 3. Definir ambas as velocidades no comando
            twist_msg.linear.x = linear_speed
            twist_msg.angular.z = angular_speed_correction
        
            self.publisher_.publish(twist_msg)

            # --- LÓGICA DE DISTÂNCIA (inalterada) ---
            if self.current_position:
                distance_traveled = math.sqrt(
                    (self.current_position.x - start_position.x)**2 +
                    (self.current_position.y - start_position.y)**2
                )
        
            # Log aprimorado
            self.get_logger().info(
                f'Dist: {distance_traveled:.2f}m | Erro Ângulo: {math.degrees(angle_error):.2f}° | Correção ω: {angular_speed_correction:.3f} rad/s',
                throttle_duration_sec=1
            )

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
        plt.plot(self.plot_x_real, self.plot_y_real, color='green', linestyle='-', linewidth=2, label='Trajeto Real (Odometria Gazebo)')
        arrow_skip = 10 
        arrow_length = 0.1 
        for i in range(0, len(self.plot_x_real), arrow_skip):
            dx_arrow = arrow_length * np.cos(self.plot_teta_real[i])
            dy_arrow = arrow_length * np.sin(self.plot_teta_real[i])
            plt.arrow(self.plot_x_real[i], self.plot_y_real[i], dx_arrow, dy_arrow, 
                      head_width=0.02, head_length=0.03, fc='darkgreen', ec='darkgreen', 
                      length_includes_head=True, alpha=0.7)
        plt.xlabel('Posição X (m)')
        plt.ylabel('Posição Y (m)')
        plt.title('Trajetória: Real (Gazebo) ')
        plt.grid(True)
        plt.axis('equal') 
        plt.legend()
        # Modifique o caminho se necessário
        plt.savefig('/home/user/trajeto_gazebo_lqr.png') 
        plt.show()
        self.get_logger().info("Gráfico de trajetória salvo em /home/user/trajeto_gazebo_lqr.png")

def main(args=None):
    rclpy.init(args=args)        
    parser = argparse.ArgumentParser(description='Controla o TurtleBot3 com LQR')
    
    parser.add_argument(
        '--control', 
        type=str, 
        default='L', 
        choices=['N', 'L'],
        help="Define se o controle utilizará LQR (L) ou não (N 'opcional')."
    )

    known_args, _ = parser.parse_known_args()

    trilha = [
        [  0, 2.0], [ 45, 1.0], [ -90, 1.0], [ 45, 1.0], [ 90, 3.0], [ 90, 2.0],
        [ 45, 2.0], [ -45, 1.0], [ -90, 1.5], [ -90, 1.0], [ 45, 2.0], [ -45, 2.0]
    ]
    
    turtle_controller = TurtleLQRController()
    time.sleep(6.0) 
    option=known_args.control
    if(option=='L'):
        turtle_controller.get_logger().info('Controle LQR ligado')
    else:
        turtle_controller.get_logger().info('Controle Simples')
                  
    turtle_controller.follow_track(trilha, option)
    turtle_controller.plot_trajectory_comparison()
    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()