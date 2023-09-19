import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'rname',
            default_value='laikago',
            description='Robot name'
        ),

        launch.actions.LogInfo(
            condition=launch.conditions.UnlessCondition(LaunchConfiguration('rname')),
            value="No robot name provided, using the default 'laikago'."
        ),

        launch.actions.LogInfo(
            condition=launch.conditions.IfCondition(LaunchConfiguration('rname')),
            value="Using provided robot name: '" + LaunchConfiguration('rname') + "'."
        ),

        launch.actions.SetLaunchConfiguration(
            name='robot_name',
            value=LaunchConfiguration('rname')
        ),

        # You can uncomment and add your ROS 2 nodes here, similar to the ROS 1 <node> tag.
    ])
