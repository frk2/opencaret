from launch import LaunchDescription
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    current_path = os.path.dirname(os.path.realpath(__file__))

    print(current_path)

    """Launch a talker and a listener."""
    return LaunchDescription([
        launch_ros.actions.Node(
            package='canoc', node_executable='transceiver', output='screen'),
        launch_ros.actions.Node(
            package='radar', node_executable='radar_controller', output='screen'),
        launch_ros.actions.Node(
            package='controls', node_executable='longitudinal_control', output='screen'),
        launch_ros.actions.Node(
            package='vehicle', node_executable='kia_soul_driver', output='screen'),

#        launch_ros.actions.Node(
#            package='planner', node_executable='planner', output='screen'),
        launch_ros.actions.Node(
            package='robot_state_publisher', node_executable='robot_state_publisher',
            arguments=[os.path.join(current_path,'../../../data/kia_soul/robot_description.urdf')]
        ),
        launch_ros.actions.Node(
            package='tf2_ros', node_executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
        ),
        launch_ros.actions.Node(
            package='tf2_ros', node_executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'front_camera_link', 'zed_left_camera']
        ),
        launch_ros.actions.Node(
            package='tf2_ros', node_executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'front_camera_link', 'zed_depth_camera']
        ),

        launch_ros.actions.Node(
            package='radar', node_executable='viz', output='screen')
    ])
