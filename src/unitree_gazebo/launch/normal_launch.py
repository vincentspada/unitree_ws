import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import ThisLaunchFileDir
from launch.actions import LogInfo

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'wname',
            default_value='earth',
            description='Name of the Gazebo world to load'
        ),

        DeclareLaunchArgument(
            'rname',
            default_value='laikago',
            description='Name of the robot model'
        ),

        DeclareLaunchArgument(
            'paused',
            default_value='true',
            description='Start Gazebo in a paused state'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulated time in ROS'
        ),

        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Launch the Gazebo GUI'
        ),

        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Start Gazebo in headless mode'
        ),

        DeclareLaunchArgument(
            'debug',
            default_value='false',
            description='Enable Gazebo debugging'
        ),

        DeclareLaunchArgument(
            'user_debug',
            default_value='false',
            description='Enable user debugging'
        ),

        LogInfo(
            condition=IfCondition(LaunchConfiguration('user_debug')),
            format=["Debug mode will hang up the robot, use 'true' or 'false' to switch it."]
        ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                [
                    ThisLaunchFileDir(),
                    '/../../gazebo_ros/launch/empty_world.launch.py'
                ]
            ),
            launch_arguments={
                'world_name': LaunchConfiguration('wname'),
                'debug': LaunchConfiguration('debug'),
                'gui': LaunchConfiguration('gui'),
                'paused': LaunchConfiguration('paused'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'headless': LaunchConfiguration('headless')
            }.items(),
        ),

        DeclareLaunchArgument(
            'robot_path',
            default_value='$(find $(arg rname)_description)',
            description='Path to the robot description package'
        ),

        DeclareLaunchArgument(
            'dollar',
            default_value='$',
            description='Dollar sign symbol'
        ),

        LogInfo(
            condition=IfCondition(LaunchConfiguration('user_debug')),
            format=["Loading the URDF into the ROS Parameter Server..."]
        ),

        LogInfo(
            condition=IfCondition(LaunchConfiguration('user_debug')),
            format=[
                "Command: $(arg dollar)($(arg robot_path))/xacro/robot.xacro",
                "DEBUG:=$(arg user_debug)"
            ]
        ),

        DeclareLaunchArgument(
            'rname_gazebo',
            default_value='$(arg rname)_gazebo',
            description='Name of the robot model in Gazebo'
        ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                [
                    ThisLaunchFileDir(),
                    '/../../gazebo_ros/launch/spawn_model.launch.py'
                ]
            ),
            launch_arguments={
                'model': LaunchConfiguration('rname_gazebo'),
                'urdf': LaunchConfiguration('robot_description'),
                'x': '0.0',
                'y': '0.0',
                'z': '0.6',
                'unpause': 'true'
            }.items(),
        ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                [
                    ThisLaunchFileDir(),
                    '/../../controller_manager/launch/spawner_launch.py'
                ]
            ),
            launch_arguments={
                'package': 'unitree_controller',
                'executable': 'spawner',
                'namespace': LaunchConfiguration('rname_gazebo'),
                'output': 'screen',
                'arguments': [
                    'joint_state_controller',
                    'FL_hip_controller',
                    'FL_thigh_controller',
                    'FL_calf_controller',
                    'FR_hip_controller',
                    'FR_thigh_controller',
                    'FR_calf_controller',
                    'RL_hip_controller',
                    'RL_thigh_controller',
                    'RL_calf_controller',
                    'RR_hip_controller',
                    'RR_thigh_controller',
                    'RR_calf_controller'
                ]
            }.items(),
        ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                [
                    ThisLaunchFileDir(),
                    '/../../robot_state_publisher/launch/state_publisher_launch.py'
                ]
            ),
            launch_arguments={
                'output': 'screen'
            }.items(),
        ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                [
                    ThisLaunchFileDir(),
                    '/../../unitree_controller/launch/set_ctrl.launch.py'
                ]
            ),
            launch_arguments={
                'rname': LaunchConfiguration('rname')
            }.items(),
        ),
    ])
