from launch import LaunchDescription
import launch_ros.actions
import os

def generate_launch_description():

    current_path = os.path.dirname(os.path.realpath(__file__))

    """Launch a talker and a listener."""
    return LaunchDescription([
        launch_ros.actions.Node(
            package='vehicle', node_executable='kia_soul_driver', output='screen'),
        launch_ros.actions.Node(
            package='radar', node_executable='radar_controller', output='screen'),
        launch_ros.actions.Node(
            package='canoc', node_executable='transceiver', arguments=['can1'], output='screen')
    ])
