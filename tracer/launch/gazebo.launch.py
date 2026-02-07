import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths
    pkg_path = '/home/deen/ros2_ws/src/tracer'
    world_path = os.path.join(pkg_path, 'world', 'tracer.sdf')
    urdf_path = os.path.join(pkg_path, 'urdf', 'tracer.urdf')

    # 1. Load URDF content
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # 2. Gazebo Sim Launch
    # Uses the ros_gz_sim helper to start Gazebo with your world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    # 3. Robot State Publisher (Essential for TFs)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # 4. Spawn the Robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'tracer_drone',
            '-string', robot_desc,
            '-z', '0.5' # Spawn slightly above ground
        ],
        output='screen',
    )

    # 5. The Bridge (Clock, Joint States, and TF)
    # Mapping: /gz_topic@ros_msg_type[gz_msg_type
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            
            # Joint States
            '/world/tracer/model/tracer_drone/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
            
            # Twist Command (Note the corrected spelling and removed '}')
            '/tracer/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',

            # TF (Transformations)
            '/model/tracer_drone/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',

            # Unpredictable Car Command (for the car driver node)
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            
            # Camera Pitch Command control
            '/model/tracer/camera/pitch_cmd@std_msgs/msg/Float64@ignition.msgs.Double',

            # Odometry of the baselink to track the orientation and control camera pitch
            '/model/tracer/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',


            # Camera Image (if you want to visualize it in ROS 2 RViz or similar)
            '/camera@sensor_msgs/msg/Image[ignition.msgs.Image'
        ],
        # remappings=[
        #     ('/cmd_vel', '/up_car/cmd_vel')
        # ],
        output='screen'
    )

    # 2. Your Gimbal Stabilizer Python Node
    # Replace 'your_package_name' with the actual name of your ROS 2 package
    stabilizer_node = Node(
        package='tracer',
        executable='cam_stabiliser',
        name='gimbal_stabilizer',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    detector_node = Node(
        package='tracer',
        executable='detector',
        name='red_car_detector',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        stabilizer_node,
        detector_node
    ])