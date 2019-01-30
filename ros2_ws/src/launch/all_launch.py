from launch import LaunchDescription
import launch_ros.actions
import os

def generate_launch_description():

    current_path = os.path.dirname(os.path.realpath(__file__))

    """Launch a talker and a listener."""
    return LaunchDescription([
        launch_ros.actions.Node(
            package='radar', node_executable='radar_controller', output='screen', arguments=['can1']),
        launch_ros.actions.Node(
            package='controls', node_executable='longitudinal_control', output='screen'),
        launch_ros.actions.Node(
            package='vehicle', node_executable='kia_soul_driver', output='screen'),
        launch_ros.actions.Node(
            package='planner', node_executable='planner', output='screen'),
        launch_ros.actions.Node(
            package='tracker', node_executable='obstacle_tracker', output='screen'),
        launch_ros.actions.Node(
            package='canoc', node_executable='transceiver', arguments=['can0'], output='screen'),
        # launch_ros.actions.Node(
        #     package='controls', node_executable='razor_imu', arguments=['/dev/ttyACM0'], output='screen'),
        # launch_ros.actions.Node(
        #     package='planner', node_executable='fake_ego', output='screen')
    ])
