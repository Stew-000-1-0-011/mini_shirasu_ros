"""mini_shirasu_node を 1 つ起動する。bridge:=true で robomas_bridge も一緒に立てる。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    params = LaunchConfiguration('params')
    bridge = LaunchConfiguration('bridge')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params',
            description='mini_shirasu_node のパラメータファイル (必須。mini_shirasu_node は既定値を持たない)',
        ),
        DeclareLaunchArgument(
            'bridge', default_value='false',
            description='robomas_bridge (robomas_plugins) も一緒に起動する',
        ),
        Node(
            package='mini_shirasu_ros',
            executable='mini_shirasu_node',
            name='mini_shirasu_node',
            parameters=[params],
            output='screen',
        ),
        ComposableNodeContainer(
            package='rclcpp_components',
            executable='component_container',
            name='robomas_container',
            namespace='',
            composable_node_descriptions=[
                ComposableNode(
                    package='robomas_plugins',
                    plugin='robomas_bridge::RobomasBridge',
                    name='robomas_bridge',
                ),
            ],
            output='screen',
            condition=IfCondition(bridge),
        ),
    ])
