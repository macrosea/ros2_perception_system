from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("camera_publish_rate_hz", default_value="30.0"),
        DeclareLaunchArgument(
            "camera_input",
            default_value="./perception_system/resources/video.mp4",
        ),
        DeclareLaunchArgument(
            "detector_model_path",
            default_value="./perception_system/models/yolov8s.engine",
        ),
    ]

    node = Node(
        package="perception_system",
        executable="perception_container",
        output="screen",
        parameters=[
            {
                "publish_rate_hz": LaunchConfiguration("camera_publish_rate_hz"),
                "input": LaunchConfiguration("camera_input"),
                "model_path": LaunchConfiguration("detector_model_path"),
            }
        ],
    )

    return LaunchDescription(args + [node])
