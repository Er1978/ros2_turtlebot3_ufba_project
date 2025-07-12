# Local: /home/turtlebot3_ws/src/turtlebot3_ufba/turtlebot3_ufba/turtle_node.py
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

# --- IMPORTAÇÕES PARA PLOTAGEM ---
import matplotlib.pyplot as plt
import numpy as np
# ---------------------------------


class PIDController:
    """Classe simples para um controlador PID."""
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0.0
        self.integral = 0.0

    def update(self, error, dt):
        # Termo Integral (com anti-windup simples)
        self.integral += error * dt

        # Termo Derivativo
        derivative = (error - self.previous_error) / dt

        # Saída do PID
        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)

        self.previous_error = error
        return output


class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller_node')

        # Define um perfil de Qualidade de Serviço (QoS) confiável para o subscriber de odometria
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Cria um publisher  para os dados de velocidade
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10) #Velocity in 3-dimensional free space broken into its linear and angular parts.

        # Cria o subscriber para receber dados de odometria
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile)
        
        self.current_position = None
        self.current_angle = None
        self.is_moving = False
        self.is_turning = False
        self.get_logger().info('Nó de Controle iniciado!')

        # --- VARIÁVEIS PARA PLOTAGEM ---
        self.plot_x_real = []
        self.plot_y_real = []
        self.plot_teta_real = []
        
        # Plotting variables for the "commanded" or "expected" path points
        # These will be initialized with the robot's actual starting position
        self.plot_x_commanded_points = [] 
        self.plot_y_commanded_points = []
        self.plot_angle_commanded_points = [] 
        
        self.plot_sample_rate = 5 # Sample every N odom messages (adjust as needed)
        self.odom_msg_count = 0
        # ---------------------------------

    # Call back para pegar a posição do robô
    def odom_callback(self, msg: Odometry):
        """Obtem os dados de odometria"""
        self.current_position = msg.pose.pose.position
        
        orientation_object = msg.pose.pose.orientation
        # Converte quaternário para ângulos de Euler (roll, pitch, yaw)
        # euler_from_quaternion retorna (roll, pitch, yaw) em radianos
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(
            [orientation_object.x, orientation_object.y, orientation_object.z, orientation_object.w])
        self.current_angle = yaw
        
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
        # Aguarda até que a primeira leitura de odometria seja recebida e esteja válida
        while self.current_position is None or self.current_angle is None:
            self.get_logger().info('Aguardando a primeira leitura de odometria...')
            rclpy.spin_once(self) # Processa callbacks para preencher current_position e current_angle
            time.sleep(0.1)
        
        # --- Sincronizar ponto inicial para plotagem de "comandada" ---
        # Adiciona o ponto de partida real do robô como o primeiro ponto comandado
        self.plot_x_commanded_points.append(self.current_position.x)
        self.plot_y_commanded_points.append(self.current_position.y)
        self.plot_angle_commanded_points.append(self.current_angle)
        # -----------------------------------------------------------------

        n = 0
        for segment in track_line:
            relative_angle_to_turn_degrees = segment[0] 
            distance_to_move = segment[1]
            n = n + 1
            self.get_logger().info(f'\n--- Percorrendo segmento {str(n)}. Rotação Relativa: {relative_angle_to_turn_degrees} º Distância: {distance_to_move}m ---')

            # --- Adicionar pontos comandados para plotagem ANTES de cada ação ---
            # Primeiro, o ponto esperado APÓS o giro
            # Usa o current_angle *antes* de iniciar o giro para calcular o alvo
            initial_angle_for_this_turn = self.current_angle # Capture o ângulo atual antes do giro
            
            # Calcula o ângulo esperado após o giro relativo
            expected_angle_after_turn = self.normalize_angle(initial_angle_for_this_turn + math.radians(relative_angle_to_turn_degrees))
            
            # Adiciona o ponto final do giro (mesma posição X,Y, novo ângulo)
            self.plot_x_commanded_points.append(self.current_position.x)
            self.plot_y_commanded_points.append(self.current_position.y)
            self.plot_angle_commanded_points.append(expected_angle_after_turn) # Adiciona o ângulo esperado após o giro

            if controller_type == 'N':
                self.turn_relative_angle(relative_angle_to_turn_degrees, 0.3)
            elif controller_type == 'P':
                # No PID, o target_angle_degrees já é o "relativo" que vira absoluto dentro da função
                self.turn_to_angle_pid(relative_angle_to_turn_degrees, Kp=1.5, Ki=0.1, Kd=0.5) 

            time.sleep(0.5) # Pequena pausa para estabilizar antes de mover

            # Depois, o ponto esperado APÓS o movimento linear
            # Usar o self.current_angle *após* o giro para calcular a trajetória linear
            # Usamos a pose *real* atual para o cálculo do ponto comandado, pois é de onde o robô realmente está partindo para o movimento linear.
            expected_final_x = self.current_position.x + distance_to_move * math.cos(self.current_angle)
            expected_final_y = self.current_position.y + distance_to_move * math.sin(self.current_angle)
            
            # Adiciona o ponto final do movimento (nova posição X,Y, mesmo ângulo)
            self.plot_x_commanded_points.append(expected_final_x)
            self.plot_y_commanded_points.append(expected_final_y)
            self.plot_angle_commanded_points.append(self.current_angle) # Angulo não muda durante movimento linear

            self.move_robot(distance_to_move, 0.2)
            
            time.sleep(0.5) # Pequena pausa para estabilizar depois de mover
        
        self.get_logger().info('Caminho finalizado')

    def turn_to_angle_pid(self, relative_angle_degrees, Kp=1.5, Ki=0.1, Kd=0.5, tolerance_degrees=0.05):
        """
        Gira o robô por um ângulo RELATIVO (a partir da orientação atual) usando o controlador PID.
        """
        if self.is_turning:
            self.get_logger().warn('Robo já está girando, ignorando comando.')
            return

        self.is_turning = True
        
        # Guarda a orientação inicial do giro para calcular o alvo absoluto
        # self.current_angle já é atualizado pelo odom_callback
        initial_angle_at_start_of_turn = self.current_angle 

        relative_angle_rad = math.radians(relative_angle_degrees)
        tolerance_rad = math.radians(tolerance_degrees)
        pid = PIDController(Kp, Ki, Kd)
        
        # Calcula o ângulo ABSOLUTO que o robô deve atingir
        target_absolute_angle_rad = self.normalize_angle(initial_angle_at_start_of_turn + relative_angle_rad)
        
        self.get_logger().info(f'Iniciando rotação RELATIVA de {relative_angle_degrees}° (De: {math.degrees(initial_angle_at_start_of_turn):.2f}° Para: {math.degrees(target_absolute_angle_rad):.2f}°)')
        
        last_time = self.get_clock().now().nanoseconds / 1e9
        
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01) # Processa callbacks para atualizar o self.current_angle
            
            # Garante que current_angle foi atualizado antes de usar
            if self.current_angle is None:
                self.get_logger().warn("current_angle ainda é None na turn_to_angle_pid. Aguardando...")
                time.sleep(0.01)
                continue

            current_time = self.get_clock().now().nanoseconds / 1e9
            dt = current_time - last_time
            last_time = current_time

            if dt <= 0: # Evita divisão por zero ou dt negativo
                continue

            # Calcula o erro, garantindo que seja o caminho mais curto até o ALVO ABSOLUTO
            error = self.normalize_angle(target_absolute_angle_rad - self.current_angle)

            # Condição de parada: o erro é menor que a nossa tolerância
            if abs(error) <= tolerance_rad:
                self.get_logger().info('Alvo de rotação atingido com sucesso.')
                break

            # Calcula a velocidade angular a ser aplicada
            angular_speed = pid.update(error, dt)

            # Saturação: limita a velocidade para proteger o robô (assumi 2.8 rad/s como limite do Burger)
            max_angular_speed = 2.8 # Max angular speed for Turtlebot3 Burger
            angular_speed = max(min(angular_speed, max_angular_speed), -max_angular_speed)

            # Publica o comando de velocidade
            twist_msg = Twist()
            twist_msg.angular.z = angular_speed
            self.publisher_.publish(twist_msg)

            self.get_logger().info(
                f'Angulo atual: {math.degrees(self.current_angle):.2f}° | Erro: {math.degrees(error):.2f}° | Vel. Aplicada: {angular_speed:.3f} rad/s', 
                throttle_duration_sec=0.5)
            
        # Para o robô completamente ao final da tarefa
        self.stop_robot()
        self.is_turning = False

    def turn_relative_angle(self, relative_angle_degrees, angle_speed=0.1, tolerance_degrees=1.0):
        """Gira o robo por um angulo RELATIVO (a partir da orientação atual) sem usar PID."""
        if self.is_turning:
            self.get_logger().warn('Robo já está girando, ignorando comando.')
            return

        self.is_turning = True
        
        # Guarda a orientação inicial do giro para calcular o alvo absoluto
        initial_angle_at_start_of_turn = self.current_angle # Base do giro relativo

        relative_angle_rad = math.radians(relative_angle_degrees)
        tolerance_rad = math.radians(tolerance_degrees)
        
        # Calcula o ângulo ABSOLUTO que o robô deve atingir
        target_absolute_angle_rad = self.normalize_angle(initial_angle_at_start_of_turn + relative_angle_rad)

        self.get_logger().info(f'Iniciando giro RELATIVO de {relative_angle_degrees}° (De: {math.degrees(initial_angle_at_start_of_turn):.2f}° Para: {math.degrees(target_absolute_angle_rad):.2f}°)')

        # Limita a velocidade angular
        max_angular_speed = 2.8 
        angle_speed = max(min(angle_speed, max_angular_speed), -max_angular_speed)
        
        # Se o ângulo relativo for 0 ou muito pequeno, saia imediatamente após parar o robô
        if abs(relative_angle_degrees) < 0.1: # Considera 0 se for muito pequeno
            self.stop_robot()
            self.get_logger().info(f'Giro relativo de 0° completo. Orientação final: {math.degrees(self.current_angle):.2f}°')
            self.is_turning = False
            return
            
        twist_msg = Twist()
        twist_msg.linear.x = 0.0 # Sem movimento linear
        
        # Loop de giro
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01) # Processa callbacks para atualizar current_angle

            if self.current_angle is None:
                self.get_logger().warn("current_angle ainda é None na turn_relative_angle. Aguardando...")
                time.sleep(0.01)
                continue

            # Calcula a diferença angular em cada iteração (erro para o controle)
            current_angle_diff = self.normalize_angle(target_absolute_angle_rad - self.current_angle)
            
            # Determina a direção do giro com base no erro (para PID simples como o seu)
            turn_direction = 1 if current_angle_diff > 0 else -1

            # Satura a velocidade angular e aplica a direção
            cmd_angular_speed = min(abs(current_angle_diff) * 1.5, angle_speed) * turn_direction # 1.5 é um P-gain
            twist_msg.angular.z = cmd_angular_speed

            # Condição de parada: se a diferença (abs) for menor que a tolerância
            # ou se o robô ultrapassou o alvo (o sinal do erro mudou ou está muito próximo de zero)
            if abs(current_angle_diff) <= tolerance_rad:
                self.get_logger().info('Alvo de rotação atingido com sucesso.')
                break
            
            # Publica o comando de velocidade
            self.publisher_.publish(twist_msg)

            self.get_logger().info(f'Angulo atual: {math.degrees(self.current_angle):.2f}° | Erro: {math.degrees(current_angle_diff):.2f}° | Vel. Aplicada: {twist_msg.angular.z:.3f} rad/s', 
                                    throttle_duration_sec=0.5)

        # Para o robô completamente ao final da tarefa
        self.stop_robot()
        self.is_turning = False
        self.get_logger().info(f'Giro relativo completo. Orientação final: {math.degrees(self.current_angle):.2f}°')


    def move_robot(self, target_distance, linear_speed=0.2, tolerance_meters=0.05):
        """Move o robô para frente por uma distância específica e depois pára."""
        linear_speed = min(linear_speed, 0.22) # Max linear speed for Turtlebot3 Burger
        if self.is_moving:
            self.get_logger().warn('Robo já está se deslocando, ignorando comando.')
            return
        
        self.is_moving = True
        
        # Guarda a posição inicial do movimento
        start_position_x = self.current_position.x
        start_position_y = self.current_position.y
        
        distance_traveled = 0.0

        self.get_logger().info(f'Iniciando movimento. Distância a percorrer: {target_distance:.2f}m a {linear_speed:.2f}m/s.')

        twist_msg = Twist()
        twist_msg.linear.x = linear_speed
        twist_msg.angular.z = 0.0 # Sem giro

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            if self.current_position is None:
                self.get_logger().warn("current_position ainda é None na move_robot. Aguardando...")
                time.sleep(0.01)
                continue

            # Calcula a distância percorrida resultante de X Y
            distance_traveled = math.sqrt(
                (self.current_position.x - start_position_x)**2 +
                (self.current_position.y - start_position_y)**2
            )

            # Condição de parada: a distância percorrida é maior ou igual à distância alvo (com tolerância)
            if distance_traveled >= target_distance - tolerance_meters:
                self.get_logger().info(f'Distância de {target_distance:.2f}m atingida. Percorrido: {distance_traveled:.2f}m.')
                break

            # Publica a velocidade enquanto a distância não for atingida
            self.publisher_.publish(twist_msg)

            self.get_logger().info(f'Distância percorrida: {distance_traveled:.2f}m', throttle_duration_sec=1)
        
        # Parar o robô
        self.stop_robot()
        self.is_moving = False

    def stop_robot(self):
        """Pára o robô enviando um sinal zerado"""
        stop_msg = Twist()
        # Garante que todos os componentes são zero para parar completamente
        stop_msg.linear.x = 0.0
        stop_msg.linear.y = 0.0
        stop_msg.linear.z = 0.0
        stop_msg.angular.x = 0.0
        stop_msg.angular.y = 0.0
        stop_msg.angular.z = 0.0
        self.publisher_.publish(stop_msg)
        self.is_moving = False        
        self.is_turning = False

    # --- FUNÇÃO DE PLOTAGEM AQUI ---
    def plot_trajectory_comparison(self):
        self.get_logger().info("Gerando gráfico de trajetória...")
        plt.figure(figsize=(12, 10))
        
        # Plotar trajetória REAL (do Gazebo)
        plt.plot(self.plot_x_real, self.plot_y_real, color='green', linestyle='-', linewidth=2, label='Trajeto Real (Odometria Gazebo)')
        # Adicionar setas para a orientação REAL
        arrow_skip_real = 10 
        arrow_length = 0.1 
        for i in range(0, len(self.plot_x_real), arrow_skip_real):
            dx_arrow = arrow_length * np.cos(self.plot_teta_real[i])
            dy_arrow = arrow_length * np.sin(self.plot_teta_real[i])
            plt.arrow(self.plot_x_real[i], self.plot_y_real[i], dx_arrow, dy_arrow, 
                      head_width=0.02, head_length=0.03, fc='darkgreen', ec='darkgreen', 
                      length_includes_head=True, alpha=0.7)

        # Plotar Trajetória Esperada/Comandada (pontos de giro/movimento)
        # É importante que todas essas listas tenham o mesmo comprimento
        # O loop deve ir até o menor comprimento para evitar IndexErrors
        min_len_commanded = min(len(self.plot_x_commanded_points), 
                                len(self.plot_y_commanded_points), 
                                len(self.plot_angle_commanded_points))

        plt.plot(self.plot_x_commanded_points[:min_len_commanded], self.plot_y_commanded_points[:min_len_commanded], 
                 color='red', linestyle='--', marker='o', markersize=5, 
                 linewidth=1, label='Trajetória Comandada (Pontos Alvo)')
        
        # Adicionar setas para a orientação Comandada (apenas nos pontos marcados)
        arrow_skip_cmd = 1 # Para plotar uma seta em cada ponto comandado
        for i in range(0, min_len_commanded, arrow_skip_cmd): 
            x_cmd = self.plot_x_commanded_points[i]
            y_cmd = self.plot_y_commanded_points[i]
            teta_cmd = self.plot_angle_commanded_points[i]

            dx_cmd_arrow = arrow_length * np.cos(teta_cmd)
            dy_cmd_arrow = arrow_length * np.sin(teta_cmd)
            plt.arrow(x_cmd, y_cmd, dx_cmd_arrow, dy_cmd_arrow,
                      head_width=0.02, head_length=0.03, fc='darkred', ec='darkred',
                      length_includes_head=True, alpha=0.8, zorder=5) # zorder para garantir que a seta apareça acima da linha

        plt.xlabel('Posição X (m)')
        plt.ylabel('Posição Y (m)')
        plt.title('Comparação de Trajetória: Real (Gazebo) vs. Comandada')
        plt.grid(True)
        plt.axis('equal') 
        plt.legend()
        plt.savefig('/home/trajeto_comparado.png')
        self.get_logger().info("Gráfico de trajetória comparada salvo em /home/trajeto_comparado.png")


def main(args=None):
    rclpy.init(args=args) 
    parser = argparse.ArgumentParser(description='Controla o TurtleBot3 com ou sem PID')
    
    parser.add_argument(
        '--control', 
        type=str, 
        default='N', 
        choices=['N', 'P'],
        help="Define se o controle utilizará PID (P) ou não (N 'opcional')."
    )

    known_args, _ = parser.parse_known_args()

    trilha = [
        [ 0, 10.0],  # Segmento 1: Gira 0 graus, move 10m. teta=0
        [ 90, 10.0], # Segmento 2: Gira +90 graus (de 0 para 90), move 10m. teta=90
        [ 90, 6.0],  # Segmento 3: Gira +90 graus (de 90 para 180), move 6m. teta=180
        [ 0, 3.0],   # Segmento 4: Gira 0 graus (mantém 180), move 3m. teta=180
        [-90, 3.0],  # Segmento 5: Gira -90 graus (de 180 para 90), move 3m. teta=90
        [-90, 3.0],  # Segmento 6: Gira -90 graus (de 90 para 0), move 3m. teta=0
        [-90, 7.0],  # Segmento 7: Gira -90 graus (de 0 para -90), move 7m. teta=-90
        [ 90, 4.0]   # Segmento 8: Gira +90 graus (de -90 para 0), move 4m. teta=0
    ]
    
    turtle_controller = TurtleController()
    
    # Aguarda o robô iniciar e receber a primeira odometria
    while turtle_controller.current_position is None or turtle_controller.current_angle is None:
        rclpy.spin_once(turtle_controller, timeout_sec=0.1)
        turtle_controller.get_logger().info('Aguardando a pose inicial do robô...')

    # Sincroniza o primeiro ponto da trajetória comandada com a pose inicial real
    turtle_controller.plot_x_commanded_points.append(turtle_controller.current_position.x)
    turtle_controller.plot_y_commanded_points.append(turtle_controller.current_position.y)
    turtle_controller.plot_angle_commanded_points.append(turtle_controller.current_angle)

    turtle_controller.get_logger().info(f"Posição inicial do robô (Real): x={turtle_controller.current_position.x:.2f}, y={turtle_controller.current_position.y:.2f}, teta={math.degrees(turtle_controller.current_angle):.2f}°")

    option = known_args.control
    if option == 'N':
        turtle_controller.get_logger().info('Controle Simples')
    elif option == 'P':
        turtle_controller.get_logger().info('Controle PID ligado')
    
    # Executa a trilha
    turtle_controller.follow_track(trilha, option)

    # --- CHAMA A FUNÇÃO DE PLOTAGEM AQUI ---
    turtle_controller.plot_trajectory_comparison()

    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()