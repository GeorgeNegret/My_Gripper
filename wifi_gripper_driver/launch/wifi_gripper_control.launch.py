
import launch
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.conditions import IfCondition, UnlessCondition
import launch_ros
import os


def generate_launch_description():

    prefix = LaunchConfiguration("prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    launch_rviz = LaunchConfiguration("launch_rviz")

    description_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="my_gripper_description"
    ).find("my_gripper_description")

    default_model_path = os.path.join(
        description_pkg_share, "urdf", "wifi_gripper.xacro"
    )
    default_rviz_config_path = os.path.join(
        description_pkg_share, "rviz", "urdf.rviz"
    )

    pkg_share = launch_ros.substitutions.FindPackageShare(
        package="wifi_gripper_driver"
    ).find("wifi_gripper_driver")


    args = []
    args.append(
        launch.actions.DeclareLaunchArgument(
            name="model",
            default_value=default_model_path,
            description="Absolute path to gripper URDF file",
        )
    )
    args.append(
        launch.actions.DeclareLaunchArgument(
            name="rvizconfig",
            default_value=default_rviz_config_path,
            description="Absolute path to rviz config file",
        )
    )
    args.append(
        launch.actions.DeclareLaunchArgument(
            name="use_fake_hardware", 
            default_value="false", 
            description="Use fake controller to simulate gripper?",
        )
    )
    args.append(
        launch.actions.DeclareLaunchArgument(
            name="fake_sensor_commands", 
            default_value="false", 
            description="Use fake_sensor_commands to simulate commands?",
        )
    )
    args.append(
        launch.actions.DeclareLaunchArgument(
            name="prefix",
            default_value="",
            description="Prefix of the joint_names, if changed also change the controller configuration.",
        )
    )
    args.append(
        launch.actions.DeclareLaunchArgument(
            name:="launch_rviz", 
            default_value="true", 
            description="Launch RViz?")
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            LaunchConfiguration("model"),
            " ",
            "use_sim:=false",
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "fake_sensor_commands:=",
            fake_sensor_commands,
            " ",
        ]
    )
    robot_description_param = {
        "robot_description": launch_ros.parameter_descriptions.ParameterValue(
            robot_description_content, value_type=str
        )
    }
    
    initial_joint_controllers = PathJoinSubstitution(
        [pkg_share, "config", "wifi_gripper_controllers.yaml"]
    )

    # define update rate
    update_rate_config_file = PathJoinSubstitution(
        [pkg_share, "config", "wifi_gripper_update_rate.yaml"]
    )

    micro_ros_agent_node = launch_ros.actions.Node(
    	package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent",
        condition=UnlessCondition(use_fake_hardware),
        arguments=["udp4", "-p", "9999", "-v6"],
        output="screen",
    )

    control_node = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description_param,
            # update_rate_config_file,
            initial_joint_controllers,
        ],
        output="screen",
        # condition=IfCondition(use_fake_hardware),
    )
    
    wifi_gripper_ros2_control_node = launch_ros.actions.Node(
        package="wifi_gripper_driver",
        executable="wifi_gripper_ros2_control_node",
        parameters=[
            robot_description_param, 
            # update_rate_config_file, 
            initial_joint_controllers
        ],
        output="screen",
        condition=UnlessCondition(use_fake_hardware),
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description_param],
    )

    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        condition=IfCondition(launch_rviz),
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    joint_state_broadcaster_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    wifi_gripper_controller_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["wifi_gripper_controller", "-c", "/controller_manager"],
    )


    nodes = [
        # micro_ros_agent_node,
        control_node,
        # wifi_gripper_ros2_control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        wifi_gripper_controller_spawner,
        rviz_node,
    ]

    return launch.LaunchDescription(args + nodes)
