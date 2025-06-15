# Local: /home/turtlebot3_ws/src/turtlebot3_ufba/turtlebot3_ufba/turtle_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy, QoSHistoryPolicy
import math
import time
import tf_transformations


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

    #Call back para pegar a posição do robô
    def odom_callback(self,msg: Odometry):
        """Obtem os dados de odometria"""
        self.current_position = msg.pose.pose.position
        #orientation_q= tf_transformations.euler_from_quaternion(msg.pose.pose.orientation)
        orientation_object=msg.pose.pose.orientation
        orientation_q= tf_transformations.euler_from_quaternion([orientation_object.x,orientation_object.y,orientation_object.z,orientation_object.w])
        self.current_angle= orientation_q[2]
        
        # self.get_logger().info(f"Posição atual: x={self.current_position.x:.2f}, y={self.current_position.y:.2f}")

    def normalize_angle(self, angle):
        """Normaliza um ângulo para o intervalo [-pi, pi]."""
        return (angle + math.pi) % (2 * math.pi) - math.pi     

    def follow_track(self, track_line):
        """Percorre a trilha com as curvas dadas no vetor track_line"""
        # Aguarda até que a primeira leitura de odometria seja recebida
        while self.current_position is None:
            self.get_logger().info('Aguardando a primeira leitura de odometria...')
            rclpy.spin_once(self)
            time.sleep(0.1)
        
       
        n_seg=len(track_line)
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
            self.turn_to_angle(angle,0.3)
            #Guarda a ultima orientação para proximos giros
            #last_angle_rad=self.current_angle 
            self.move_robot(distance,0.2)


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


def main(args=None):
        
    #Cria um vetor com o giro e deslocamentos absolutos p/ cada trecho
    trilha = [
        [  0, 10.0], 
        [ 90, 10.0], 
        [ 90, 6.0], 
        [ 90, 3.0], 
        [ 90, 3.0], 
        [-90, 3.0], 
        [-90, 7.0], 
        [ 90, 4.0] 
        ]


    rclpy.init(args=args)
    turtle_controller = TurtleController()
    #rclpy.spin(turtle_controller)
    time.sleep(2.0) 
    turtle_controller.follow_track(trilha)
    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()