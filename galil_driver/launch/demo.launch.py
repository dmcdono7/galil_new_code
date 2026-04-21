
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    ld = LaunchDescription()

    arg_jsp_gui = DeclareLaunchArgument(
        name='jsp_gui', 
        default_value='false', 
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui'
    )
    
    arg_rviz = DeclareLaunchArgument(
        name='rviz', 
        default_value='false', 
        choices=['true', 'false'],
        description='Flag to enable rviz'
    )

    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    arg_name = DeclareLaunchArgument(
            'name',
            default_value='galil',
            description='Name of the Galil system'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rate = LaunchConfiguration('rate', default=50.0)  # Hz, default is 10 so we're increasing that a bit. 

    # Define urdf description
    urdf_file_name = 'galil.urdf.xacro'
    urdf = os.path.join(
        get_package_share_directory('galil_description'),
        'urdf',
        urdf_file_name)

    # Define rviz path
    rviz_config = os.path.join(
        get_package_share_directory('galil_description'),
        'rviz',
        'urdf.rviz')

    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': ParameterValue(
                         Command(['xacro ', str(urdf), ' ', 'name:=', LaunchConfiguration('name')]), value_type=str),
                     'publish_frequency': rate
        }],
        arguments=[urdf]
    )

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('jsp_gui'))
    )
    joint_state_publisher_node_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('jsp_gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', str(rviz_config)],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Arguments
    ld.add_action(arg_rviz)
    ld.add_action(arg_jsp_gui)
    ld.add_action(arg_use_sim_time)
    ld.add_action(arg_name)
    # Nodes
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(joint_state_publisher_node_gui)
    ld.add_action(rviz_node)
    
    return ld
