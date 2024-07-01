from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='waffle'),
        SetEnvironmentVariable(name='LDS_MODEL', value='LDS-01'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('turtlebot3_bringup'),
                    'launch',
                    'robot.launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': 'False',
            }.items()
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom']
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'odom', 'base_footprint']
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_footprint', 'base_link']
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'laser']
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'navigation_launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': 'False',
                'autostart': 'True',
                'map': '/home/vesa/map2.yaml',
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('slam_toolbox'),
                    'launch',
                    'online_async_launch.py'
                ])
            ]),
            launch_arguments={
                #'use_sim_time': 'False',
            }.items()
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', '/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz']
        ),


    ])