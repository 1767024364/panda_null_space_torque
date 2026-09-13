from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument

# from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    headless = LaunchConfiguration("headless")

    xacro_path = PathJoinSubstitution([FindPackageShare("panda_description"), "urdf", "panda_ros2_control.urdf.xacro"])

    robot_description = Command([FindExecutable(name="xacro"), " ", xacro_path, " headless:=", headless,])

    controller_parameters = PathJoinSubstitution([FindPackageShare("panda_description"), "config", "test.yaml",])





    robot_state_publisher = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }

        ],
    )

    return  LaunchDescription(
        [   
            DeclareLaunchArgument("headless", default_value = "true", description = "无界面运行 MuJoCo "),
            robot_state_publisher,
        ]
    )