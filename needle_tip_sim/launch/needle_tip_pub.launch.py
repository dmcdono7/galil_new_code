from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Declare launch configuration/argument "period"
    period = LaunchConfiguration('period')
    period_launch_arg = DeclareLaunchArgument(
        'period',
        default_value='200'
    )
    
    # Declare launch configuration/argument "period"
    sim_tip_vals_file = LaunchConfiguration('sim_tip_vals_file')
    file_launch_arg = DeclareLaunchArgument(
        'sim_tip_vals_file',
        default_value=PathJoinSubstitution([FindPackageShare("needle_tip_sim"), "data", "insertion_points.csv"])
    )

    publisher = Node(
        package='needle_tip_sim',
        executable='pub',
        output="screen",
        parameters=[
            {"period": period},
            {"sim_tip_vals_file" : sim_tip_vals_file}
        ])

    # launch all the above
    return LaunchDescription([
        period_launch_arg,
	file_launch_arg,
        publisher
        ])
