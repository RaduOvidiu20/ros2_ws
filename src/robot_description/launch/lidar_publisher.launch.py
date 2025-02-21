import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="robot_description",  # Schimbă dacă numele pachetului este diferit
                executable="lidar_publisher.py",
                name="lidar_publisher",
                output="screen",
                parameters=[
                    {
                        "lidar_topic": "/scan",  # Modifică după necesități
                        "frame_id": "laser_frame",
                    }
                ],
            )
        ]
    )
