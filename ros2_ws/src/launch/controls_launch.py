from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='vehicle', node_executable='kia_soul_driver', output='screen'),
        launch_ros.actions.Node(
            package='canoc', node_executable='transceiver', output='screen'),
        launch_ros.actions.Node(
            package='controls', node_executable='longitudinal_control', output='screen'),
        launch_ros.actions.Node(
            package='planner', node_executable='planner', output='screen'),
        launch_ros.actions.Node(
            package='planner', node_executable='fake_ego', output='screen'),

    ])
