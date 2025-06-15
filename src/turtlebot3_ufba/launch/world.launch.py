# /home/turtlebot3_ws/src/turtlebot3_ufba/launch/world.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from geometry_msgs.msg import Quaternion
import tf_transformations

def generate_launch_description():
    pkg_name = 'turtlebot3_ufba'
    pkg_share_turtlebot3_ufba = get_package_share_directory(pkg_name)
    
    # Caminho para o nosso mundo vazio
    world_path = os.path.join(pkg_share_turtlebot3_ufba, 'worlds', 'track.sdf')

    # Ação para iniciar o Gazebo com o mundo VAZIO, ignorando a tela de início
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen',
        name='gazebo'
    )
 
    # --- A NOVA AÇÃO PARA COMANDAR A CÂMERA ---
    # Pose: {X:0, Y:0, Z:10, Roll:0, Pitch:1.57, Yaw:-1.57}
    # A orientação é convertida para Quaternion (w,x,y,z)
    orientation_q = tf_transformations.quaternion_from_euler(0, 1.5707, 1.5707) 
    x_q=orientation_q[0]
    y_q=orientation_q[1]
    z_q=orientation_q[2]
    w_q=orientation_q[3]

    set_camera_pose = ExecuteProcess(
        cmd=['gz', 'service', '-s', '/gui/move_to/pose', '--reqtype', 'gz.msgs.GUICamera', '--reptype','gz.msgs.Boolean',
             '-r',
             'name: "box", pose: {position: {x: 1,y: 4,z: 10}, orientation: {x: ' + str(x_q) + ', y: ' + str(y_q) + ', z: ' + str(z_q) + ', w: ' + str(w_q) + '}}, projection_type: "orbit"' 
            ],
        output='screen'
    )

    
    pkg_share_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    #robot_sdf_path = os.path.join(pkg_share_turtlebot3_gazebo, 'models', 'turtlebot3_burger', 'model.sdf')
    robot_sdf_path = os.path.join(pkg_share_turtlebot3_ufba, 'models', 'turtlebot3_burger_vermelho', 'model.sdf')
    
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', robot_sdf_path, '-name', 'turtlebot3_burger', '-x', '-3.0', '-y', '-1.0', '-z', '0.0'],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock', '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry', '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V', '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan', '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU', '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model', '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'],
        output='screen'
    )

    control_node = Node(
        package=pkg_name,
        executable='turtle_controller',
        name='turtle_controller_node',
        output='screen'
    )
    
    # Gatilho: espera o processo 'gazebo' iniciar, aguarda 2s e então cria TUDO
    spawn_trigger = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=gazebo,
            on_start=[
                TimerAction(
                    period=2.0,
                     actions=[set_camera_pose]
                ),
                TimerAction(
                    period=2.0,
                    actions=[
                        spawn_robot,
                        bridge,
                        control_node
                    ]
                )
               
            ]
        )
    )

    return LaunchDescription([
        gazebo,
        spawn_trigger
    ])