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
        self.current_angle=None
        self.is_moving = False
        self.is_turning=False
        self.get_logger().info('Nó de Controle iniciado!')

        
        #self.timer = self.create_timer(0.5, self.timer_callback)
        #self.get_logger().info('Nó de controle iniciado. Movendo o robô...')
        #self.counter = 0

          # --- VARIÁVEIS PARA PLOTAGEM ---
        self.plot_x_real = []
        self.plot_y_real = []
        self.plot_teta_real = []

            # Plotting variables for the "commanded" or "expected" path points
        # These are the points you TELL the robot to go to or turn to.
        #self.plot_x_commanded_points = [0.0] # Start at 0,0
        #self.plot_y_commanded_points = [0.0]
        #self.plot_angle_commanded_points = [0.0] # Start at 0
        
        self.plot_sample_rate = 5 # Sample every N odom messages (adjust as needed)
        self.odom_msg_count = 0
        # ---------------------------------

    #Call back para pegar a posição do robô
    def odom_callback(self,msg: Odometry):
        """Obtem os dados de odometria"""
        self.current_position = msg.pose.pose.position
        #orientation_q= tf_transformations.euler_from_quaternion(msg.pose.pose.orientation)
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
        # self.get_logger().info(f"Posição atual: x={self.current_position.x:.2f}, y={self.current_position.y:.2f}")

    def normalize_angle(self, angle):
        """Normaliza um ângulo para o intervalo [-pi, pi]."""
        return (angle + math.pi) % (2 * math.pi) - math.pi     

    def follow_track(self, track_line, controller_type):
        """Percorre a trilha com as curvas dadas no vetor track_line"""
        # Aguarda até que a primeira leitura de odometria seja recebida
        while self.current_position is None:
            self.get_logger().info('Aguardando a primeira leitura de odometria...')
            rclpy.spin_once(self)
            time.sleep(0.1)
        
        n=0
        last_angle_rad=0
        angle=0.00
        distance=0.00
        #Percorrendo a trilha
        for target_position in track_line:
            #last_angle_degrees=last_angle_rad*180/math.pi
            angle=target_position[0] 
            distance=target_position[1]
            n=n+1
            self.get_logger().info(f'Percorrendo segmento {str(n)}. Rotação: {angle} º Distância: {distance}m')

            # --- Adicionar pontos comandados para plotagem ANTES de cada ação ---
            # Para o giro: o ponto esperado é o mesmo, mas a orientação muda
            #self.plot_x_commanded_points.append(self.current_position.x)
            #self.plot_y_commanded_points.append(self.current_position.y)
            # Calcula o ângulo esperado após o giro relativo
            #expected_angle_after_turn = self.normalize_angle(self.current_angle + math.radians(angle))
            #self.plot_angle_commanded_points.append(expected_angle_after_turn)

            if(controller_type=='N'):
                self.turn_to_angle(angle,0.3)
                self.move_robot(distance,0.2)
            elif(controller_type=='P'):
                self.turn_to_angle_pid(angle,0.3)
                self.move_robot(distance,0.2)
        
             # Para o movimento: o ponto esperado é o ponto final do movimento
            # Calculamos o ponto final com base na orientação atual e distância
            expected_final_x = self.current_position.x + distance * math.cos(self.current_angle)
            expected_final_y = self.current_position.y + distance * math.sin(self.current_angle)
            #self.plot_x_commanded_points.append(expected_final_x)
            #self.plot_y_commanded_points.append(expected_final_y)
            




        self.get_logger().info('Caminho finalizado')

    def turn_to_angle_pid(self, target_angle_degrees, Kp=1.5, Ki=0.1, Kd=0.5, tolerance_degrees=0.05):
            """
            Gira o robô para um ângulo alvo (em graus) usando o controlador PID.
            """
            if self.is_turning:
                self.get_logger().info('Robo sendo redirecionado')
                return

            self.is_turning = True
            target_angle_rad = self.normalize_angle(math.radians(target_angle_degrees))
            tolerance_rad = math.radians(tolerance_degrees)
            pid = PIDController(Kp, Ki, Kd)
            
            self.get_logger().info(f'Iniciando rotação para {target_angle_degrees}° (Alvo em radianos: {target_angle_rad:.3f})')
            
            last_time = self.get_clock().now().nanoseconds / 1e9
            
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.01) # Processa callbacks para atualizar o self.current_angle
                
                current_time = self.get_clock().now().nanoseconds / 1e9
                dt = current_time - last_time
                last_time = current_time

                if dt <= 0: continue

                # Calcula o erro, garantindo que seja o caminho mais curto
                error = self.normalize_angle(target_angle_rad - self.current_angle)

                # Condição de parada: o erro é menor que a nossa tolerância
                if abs(error) <= tolerance_rad:
                    self.get_logger().info('Alvo de rotação atingido com sucesso.')
                    break

                # Calcula a velocidade angular a ser aplicada
                angular_speed = pid.update(error, dt)

                # Saturação: limita a velocidade para proteger o robô
                angular_speed = max(min(angular_speed, 2.0), -2.0)

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



    def turn_to_angle(self,target_angle_degrees, angle_speed=0.1):
        """Gira o robo para o angulo desejado em graus absolutos(YAW) sem usar PID"""
        # Limitador de velocidade para evitar danos a um robô real. O modelo Burger aceita no maximo 2.83 rad/s
        angle_speed = max(min(angle_speed, 2.0), -2.0)

        #Define o angulo final a partir da posição relativa
        #target_angle_rad=self.normalize_angle(math.radians(target_angle_degrees+self.current_angle))
        target_angle_rad=math.radians(target_angle_degrees+self.current_angle)
        rotated_angle=0.00

        #Guarda a orientação atual
        rotated_previous=self.current_angle
        self.get_logger().info(f'Angulo atual: {math.degrees(rotated_previous)}')
        if self.is_turning:
            self.get_logger().info('Robo sendo redirecionado')
            return

        self.is_turning=True
        
        twist_msg = Twist()
        
        #Se o angulo pedido for menor  que zero gira ao contrário
        if(target_angle_degrees<0):
            angle_speed=angle_speed * -1

        #Gira o robô sem deslocamento
        twist_msg.angular.z=angle_speed
        twist_msg.linear.x = 0.0

        while abs(rotated_angle) < abs(target_angle_rad):
            self.publisher_.publish(twist_msg)
            rclpy.spin_once(self)
            # Se existir informação do angulo armazena como rotated_angle
            if self.current_angle:
                rotated_angle = self.normalize_angle(self.current_angle - rotated_previous)
                rotated_angle_degrees= math.degrees(rotated_angle)
                self.get_logger().info(f'Robô girado em {rotated_angle_degrees:.2f}º', throttle_duration_sec=0.5)
                #time.sleep(0.05) # Pequena pausa para não sobrecarregar a CPU
                
        #Parando o giro do robo
        self.stop_robot()
        self.get_logger().info('Robô direcionado')

    def move_robot(self, target_distance, linear_speed=0.2):
        """Move o robô para frente por uma distância específica e depois pára."""
        #Velocidade maxima permitida no modelo Burger=0.2 m/s
        linear_speed=min(linear_speed,0.2)
        if self.is_moving:
            self.get_logger().warn('Robô em deslocamento.')
            return
        
        self.is_moving = True
        start_position = self.current_position
        distance_traveled = 0.0

        self.get_logger().info(f'Iniciando movimento. Distância a percorrer: {target_distance:.2f}m a {linear_speed:.2f}m/s.')

        twist_msg = Twist()
        
        #Deslocando o robo
        twist_msg.linear.x = linear_speed
        while distance_traveled < target_distance:
            # Publica a velocidade enquanto a distância não for atingida
            self.publisher_.publish(twist_msg)
            rclpy.spin_once(self)

            # Calcula a distância percorrida resultante de X Y
            if self.current_position:
                distance_traveled = math.sqrt(
                    (self.current_position.x - start_position.x)**2 +
                    (self.current_position.y - start_position.y)**2
                )

            self.get_logger().info(f'Distância percorrida: {distance_traveled:.2f}m', throttle_duration_sec=1)
            #time.sleep(0.05) # Pequena pausa para não sobrecarregar a CPU

        # Parar o robô
        self.stop_robot()
        self.get_logger().info(f'Distância de {target_distance:.2f}m atingida.')
        self.is_moving = False


    def stop_robot(self):
        """Pára o robô enviando um sinal zerado"""
        
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.linear.y = 0.0
        stop_msg.linear.z = 0.0
        stop_msg.angular.x = 0.0
        stop_msg.angular.y = 0.0
        stop_msg.angular.z = 0.0
        self.publisher_.publish(stop_msg)
        self.is_moving = False        
        self.is_turning=False
        
# --- FUNÇÃO DE PLOTAGEM AQUI ---
    def plot_trajectory_comparison(self):
        self.get_logger().info("Gerando gráfico de trajetória...")
        plt.figure(figsize=(12, 10))
        
        # Plotar trajetória REAL (do Gazebo)
        plt.plot(self.plot_x_real, self.plot_y_real, color='green', linestyle='-', linewidth=2, label='Trajeto Real (Odometria Gazebo)')
        # Adicionar setas para a orientação REAL
        arrow_skip = 10 
        arrow_length = 0.1 
        for i in range(0, len(self.plot_x_real), arrow_skip):
            dx_arrow = arrow_length * np.cos(self.plot_teta_real[i])
            dy_arrow = arrow_length * np.sin(self.plot_teta_real[i])
            plt.arrow(self.plot_x_real[i], self.plot_y_real[i], dx_arrow, dy_arrow, 
                      head_width=0.02, head_length=0.03, fc='darkgreen', ec='darkgreen', 
                      length_includes_head=True, alpha=0.7)

        # Plotar Trajetória Esperada/Comandada (pontos de giro/movimento)
        #plt.plot(self.plot_x_commanded_points, self.plot_y_commanded_points, 
        #         color='red', linestyle='--', marker='o', markersize=5, 
        #         linewidth=1, label='Trajetória Comandada (Pontos Alvo)')
        # Adicionar setas para a orientação Comandada (apenas nos pontos marcados)
        #for i in range(len(self.plot_x_commanded_points)): # Percorre todos os pontos comandados
        #    x_cmd = self.plot_x_commanded_points[i]
        #    y_cmd = self.plot_y_commanded_points[i]
        #    teta_cmd = self.plot_angle_commanded_points[i]

            #dx_cmd_arrow = arrow_length * np.cos(teta_cmd)
            #dy_cmd_arrow = arrow_length * np.sin(teta_cmd)
            #plt.arrow(x_cmd, y_cmd, dx_cmd_arrow, dy_cmd_arrow,
            #          head_width=0.02, head_length=0.03, fc='darkred', ec='darkred',
            #          length_includes_head=True, alpha=0.8, zorder=5) # zorder para garantir que a seta apareça acima da linha

        plt.xlabel('Posição X (m)')
        plt.ylabel('Posição Y (m)')
        plt.title('Trajetória: Real (Gazebo) ')
        plt.grid(True)
        plt.axis('equal') 
        plt.legend()
        plt.savefig('/home/trajeto_gazebo.png')
        plt.show()
        self.get_logger().info("Gráfico de trajetória salvo em /home/trajeto_gazebo.png")


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

    #Cria um vetor com o giro e deslocamentos absolutos p/ cada trecho
    trilha = [
        [  0, 2.0], # segmento1
        [ 45, 1.0], # segmento2
        [ -90, 1.0], # segmento3
        [ 45, 1.0], # segmento4
        [ 90, 3.0], # segmento5
        [ 90, 2.0], # segmento6
        [ 45, 2.0], # segmento7
        [ -45, 1.0], # segmento8
        [ -90, 1.5], # segmento9
        [ -90, 1.0], # segmento10
        [ 45, 2.0], # segmento11
        [ -45, 2.0] # segmento12
                          
        ]




    
    turtle_controller = TurtleController()
    #rclpy.spin(turtle_controller)
    time.sleep(6.0) 
    option=known_args.control
    if(option=='N'):
        turtle_controller.get_logger().info('Controle Simples')
    elif(option=='P'):
        turtle_controller.get_logger().info('Controle PID ligado')      
                  
    turtle_controller.follow_track(trilha, option)

# --- INSERIR A CHAMADA DA FUNÇÃO DE PLOTAGEM AQUI ---
    turtle_controller.plot_trajectory_comparison()


    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()