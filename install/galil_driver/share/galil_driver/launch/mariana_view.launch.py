
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="galil_description",
            description="Description package with robot URDF/xacro files. Usually the argument "
            "is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="galil_driver",
            description='Package with the controller\'s configuration in "config" folder. '
            "Usually the argument is not set, it enables use of a custom setup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="galil.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='""',
            description="Prefix of the joint names, useful for "
            "multi-robot setup. If changed then also joint names in the controllers' configuration "
            "have to be updated."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="galil_controllers.yaml",
            description="YAML file with the controllers configuration."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_joint_controllers",
            default_value="true",
            description="Activate loaded joint controller."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_spawner_timeout",
            default_value="10",
            description="Timeout used when spawing controllers"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="position_controller",
            description="Initially loaded robot controller"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 and Joint State Publisher gui automatically \
            with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "name",
            default_value='"galil"',
            description="Name of the Galil system",
        )
    )

    # Initialize Arguments
    description_package = LaunchConfiguration("description_package")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    
    description_file = LaunchConfiguration("description_file")
    controllers_file = LaunchConfiguration("controllers_file")

    tf_prefix = LaunchConfiguration("tf_prefix")

    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    activate_joint_controller = LaunchConfiguration("activate_joint_controllers")
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    
    gui = LaunchConfiguration("gui")
    name = LaunchConfiguration("name")
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "name:=",
            name,
            " ",
            "tf_prefix:=",
            tf_prefix,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ParameterFile(initial_joint_controllers, allow_substs=True),
        ],
        output="both",
    )

    #robot_controllers = PathJoinSubstitution(
    #    [
    #        FindPackageShare("galil_driver"),
    #        "config",
    #        "galil.yaml",
    #    ]
    #)

    # Get RViz config
    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "rviz",
            "urdf.rviz",
        ]
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    def controller_spawner(controllers, active=True):
        inactive_flags = ["--inactive"] if not active else[]
        print( controllers )
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                controller_spawner_timeout,
            ]
            + inactive_flags
            + controllers
        )

    controllers_active=["joint_state_broadcaster", "position_controller"]
    controllers_inactive=["spacenav_controller", "broyden_controller", "velocity_controller"]
    controller_spawners=[controller_spawner(controllers_active)] + [
        controller_spawner(controllers_inactive, active=False)
    ]

    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            initial_joint_controller,
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            controller_spawner_timeout,
        ],
        condition=IfCondition(activate_joint_controller),
    )
    
    initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            initial_joint_controller,
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            controller_spawner_timeout,
            "--inactive",
        ],
        condition=UnlessCondition(activate_joint_controller),
    )
    
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    #joint_state_pub_node = Node(
    #    package="joint_state_publisher_gui",
    #    executable="joint_state_publisher_gui",
    #    output="both",
    #)    
    #joint_state_broadcaster_spawner = Node(
    #    package="controller_manager",
    #    executable="spawner",
    #    arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    #)
    #robot_controller_spawner = Node(
    #    package="controller_manager",
    #    executable="spawner",
    #    arguments=["position_trajectory", "--controller-manager", "/controller_manager"],
    #    output="screen"
    #)
    #delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
    #    event_handler=OnProcessExit(
    #        target_action=joint_state_broadcaster_spawner,
    #        on_exit=[robot_controller_spawner],
    #    )
    #)
    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     condition=UnlessCondition(LaunchConfiguration('gui'))
    # )
    # joint_state_publisher_node_gui = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     condition=IfCondition(LaunchConfiguration('gui'))
    # )

    nodes = [
        control_node,
        robot_state_pub_node,
        # joint_state_publisher_node,
        # joint_state_publisher_node_gui,
        rviz_node,
        #initial_joint_controller_spawner_started,
        #initial_joint_controller_spawner_stopped,
        #joint_state_broadcaster_spawner,
        #delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,    
    ] + controller_spawners

    return LaunchDescription(declared_arguments + nodes)
