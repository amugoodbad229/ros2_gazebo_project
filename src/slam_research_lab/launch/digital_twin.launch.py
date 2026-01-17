import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_research = get_package_share_directory('slam_research_lab')
    
    slam_config = os.path.join(pkg_research, 'config', 'slam_params.yaml')
    world_file = os.path.join(pkg_research, 'worlds', 'research_world.sdf')
    robot_file = os.path.join(pkg_research, 'models', 'research_bot.sdf')

    # 1. Simulation
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # 2. Spawn Robot
    spawn_robot = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'research_bot', '-x', '0', '-y', '0', '-z', '0.5', '-file', robot_file],
        output='screen'
    )

    # 3. Bridge
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'
        ],
        parameters=[{'qos_overrides./model/research_bot.subscriber.reliability': 'reliable'}],
        output='screen'
    )

    # 4. TF
    tf_lidar = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments = ['0', '0', '0.15', '0', '0', '0', 'base_link', 'research_bot/base_link/lidar']
    )

    # 5. SLAM
    slam = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', output='screen', parameters=[slam_config]
    )

    # 6. Rviz
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen'
    )

    return LaunchDescription([gazebo, spawn_robot, bridge, tf_lidar, slam, rviz])
