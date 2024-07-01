from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()

    tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom']
    )
    ld.add_action(tf_map_odom)

    tf_odom_bfp = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'odom', 'base_footprint']
    )
    ld.add_action(tf_odom_bfp)

    """tf_bfp_bl = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_footprint', 'base_link']
    )
    ld.add_action(tf_bfp_bl)"""

    """tf_odom_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'odom', 'laser']
    )
    ld.add_action(tf_odom_laser)"""

    nav2_brup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ])
        
    )
    """,
        launch_arguments={
                'use_sim_time': 'False',
                'autostart': 'False',
                'map': 'map1.yaml'
            }.items()"""
    ld.add_action(nav2_brup)

    slam_tb = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            ])
        ])
    )
    ld.add_action(slam_tb)

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', '/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz']
    )
    ld.add_action(rviz2)


    return ld