import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'user_debug',
            default_value='false',
            description='Enable user debugging (default: false)'
        ),
        
        Node(
            package='xacro',
            executable='xacro',
            name='robot_description',
            output='screen',
            parameters=[
                {'robot_description': Command(
                    [FindPackageShare('go1_description'), 'xacro', '--inorder',
                    FindPackageShare('go1_description'), 'xacro/robot.xacro',
                    'DEBUG:=', LaunchConfiguration('user_debug')])}
            ],
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_gui': True}]
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'publish_frequency': 1000.0}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', FindPackageShare('go1_description') + '/launch/check_joint.rviz'],
        )
    ])
