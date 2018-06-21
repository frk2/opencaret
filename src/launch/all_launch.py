from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    """Launch a talker and a listener."""
    return LaunchDescription([
        launch_ros.actions.Node(
            package='canoc', node_executable='transceiver', output='screen'),
        launch_ros.actions.Node(
            package='radar', node_executable='radar_controller', output='screen'),
])