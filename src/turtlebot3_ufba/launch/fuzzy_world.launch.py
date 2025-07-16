# /home/turtlebot3_ws/src/turtlebot3_ufba/launch/fuzzy_world.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from geometry_msgs.msg import Quaternion
import tf_transformations
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    pkg_name = 'turtlebot3_ufba'
    pkg_share_turtlebot3_ufba = get_package_share_directory(pkg_name)

    declare_control_arg = DeclareLaunchArgument(
        'control',
        default_value='F',
        description='Especifica qual controle será usado (F-Fuzzy (default) ou N-Normal)'
    )
    
    world_path = os.path.join(pkg_share_turtlebot3_ufba, 'worlds', 'track.sdf')

    orientation_q = tf_transformations.quaternion_from_euler(0, 1.5707, 1.5707) 
    x_q=orientation_q[0]
    y_q=orientation_q[1]
    z_q=orientation_q[2]
    w_q=orientation_q[3]

    set_camera_pose = ExecuteProcess(
        cmd=['gz', 'service', '-s', '/gui/move_to/pose', '--reqtype', 'gz.msgs.GUICamera', '--reptype','gz.msgs.Boolean',
             '-r',
             'name: "box", pose: {position: {x: 2,y: 2,z: 6}, orientation: {x: ' + str(x_q) + ', y: ' + str(y_q) + ', z: ' + str(z_q) + ', w: ' + str(w_q) + '}}, projection_type: "orbit"' 
            ],
        output='screen'
    )

    robot_sdf_path = os.path.join(pkg_share_turtlebot3_ufba, 'models', 'turtlebot3_burger_vermelho', 'model.sdf')
    
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen',
        name='gazebo'
    )
    
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', robot_sdf_path, '-name', 'turtlebot3_burger', '-x', '0.0', '-y', '0.0', '-z', '0.0'],
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
        executable='turtle_fuzzy_controller',
        name='turtle_fuzzy_controller_node',
        output='screen',
        arguments=['--control', LaunchConfiguration('control')]
    )
    
    spawn_trigger = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=gazebo,
            on_start=[
                TimerAction(
                    period=5.0,
                     actions=[set_camera_pose]
                ),
                TimerAction(
                    period=5.0,
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
        declare_control_arg,
        gazebo,
        spawn_trigger
    ])
