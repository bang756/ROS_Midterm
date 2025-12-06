import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Resolve world file from the installed project_description package
    world_file = os.path.join(
        get_package_share_directory('project_description'),
        'models',
        'warehouse_env.sdf',
    )
    models_path = os.path.join(
        get_package_share_directory('project_description'),
        'models',
    )
    env = os.environ.copy()
    if env.get('GZ_SIM_RESOURCE_PATH'):
        env['GZ_SIM_RESOURCE_PATH'] = f"{models_path}:{env['GZ_SIM_RESOURCE_PATH']}"
    else:
        env['GZ_SIM_RESOURCE_PATH'] = models_path

    # 1) Gazebo 실행 (3번 터미널에서 하던 것 그대로) #001
    start_gazebo_server = ExecuteProcess( 
        cmd=['gz', 'sim', '-r', world_file],
        output='screen',
        emulate_tty=True,
        env=env,
    )

    # 2) ROS ↔ Gazebo 브리지 (2번 터미널에서 하던 것 그대로)
    bridge_args = [
        # Drone
        '/X4/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
        '/model/X4_GPS_RGBD/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        '/world/warehouse_env/model/X4_GPS_RGBD/link/base_link/sensor/camera_front/image@sensor_msgs/msg/Image@gz.msgs.Image',
        '/world/warehouse_env/model/X4_GPS_RGBD/link/base_link/sensor/camera_front/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',

        # TurtleBot
        '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
        '/model/turtlebot3_burger/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',

        # TF 브리지 (GZ → ROS) (PosePublisher용)
        '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
    ]

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        arguments=bridge_args,
    )

    # 3) 드론 순회 노드 (3번 터미널에서 하던 것 그대로)
    drone_patrol_node = Node(
        package='project',
        executable='drone_patrol_node',
        name='drone_patrol_commander',
        output='screen',
        # sim time 아예 안 씀
        
    )

    # 4) 터틀봇 waypoint(포크리프트) 노드
    forklift_node = Node(
        package='project',
        executable='forklift_node',          # forklift_node.py의 entrypoint 이름
        name='forklift_waypoint_commander',
        output='screen',
    )

    aruco_to_forklift_node = Node(
        package='project',
        executable='aruco_to_forklift',
        name='aruco_to_forklift',
        output='screen',
    )

    # 5) 브리지가 먼저 올라오고, 조금 있다가 두 노드 같이 실행
    delayed_drone_start = TimerAction(
        period=3.0,
        actions=[
            drone_patrol_node,
            forklift_node,
            aruco_to_forklift_node,
        ]
    )

    return LaunchDescription([
        start_gazebo_server,
        bridge_node,
        delayed_drone_start,
    ])
