import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get package path
    pkg_barista = get_package_share_directory("barista_robot_description")

    # Robot description (xacro to urdf)
    robot_desc_path = os.path.join(pkg_barista, "xacro", "barista_robot_model.urdf.xacro")
    robot_description = Command(['xacro ', robot_desc_path])

    # robot_state_publisher node
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        # parameters=[{'robot_description': robot_description,
        #              'use_sim_time': True}],
        parameters=[{
            'robot_description': Command([FindExecutable(name='xacro'), ' ', robot_desc_path]),
            'use_sim_time': True
        }],
        output='screen'
    )

    # Ignition Gazebo launcher
    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-r -v 4 empty.sdf'
        }.items()
    )

    # Spawn entity using ros_gz_sim create service
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'barista_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.2'   # <- your desired z offset here
        ],
        output='screen'
    )


    # RViz (optional)
    rviz_config_dir = os.path.join(pkg_barista, 'rviz', 'config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Bridges (example for cmd_vel and laser scan)

    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'],
        output='screen'
    )

    bridge_joint_states = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model'],
        output='screen'
    )

    bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
        output='screen'
    )

    bridge_tf = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'],
        output='screen'
    )

    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'],
        output='screen'
    )

    bridge_scan = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'],
        output='screen'
    )

    return LaunchDescription([
        ign_gazebo,
        rsp_node,
        spawn_entity,
        rviz_node,
        bridge_clock,
        bridge_joint_states,
        bridge_odom,
        bridge_tf,
        bridge_cmd_vel,
        bridge_scan
    ])
