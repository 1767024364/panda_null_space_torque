from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    headless = LaunchConfiguration("headless")

    xacro_path = PathJoinSubstitution([FindPackageShare("panda_description"), "urdf", "panda_ros2_control.urdf.xacro"])

    robot_description = Command([FindExecutable(name="xacro"), " ", xacro_path, " headless:=", headless])

    controller_parameters = ParameterFile(PathJoinSubstitution([FindPackageShare("panda_bringup"), "config", "controller.yaml"]),
                                          allow_substs=True)


    mujoco_node = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        output = "screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            },
            controller_parameters,
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package = "controller_manager",
        executable = "spawner",
        arguments =[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
        ],
        output = "screen",
    )

    null_space_controller_spawner = Node(
        package = "controller_manager",
        executable = "spawner",
        arguments = [
            "null_space_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
        ],
        output = "screen",
    )



    return LaunchDescription(
        [
            DeclareLaunchArgument(name="headless", default_value="true", description="无界面运行 mujoco"),
            mujoco_node,
            joint_state_broadcaster_spawner,
            null_space_controller_spawner,
        ]
    )