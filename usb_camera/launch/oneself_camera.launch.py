from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='usb_camera',
                plugin='usb_camera::USBCameraNode',
                name='camera1',
                parameters=[{
                    'device_path': '/dev/video0',
                    'frame_id': 'camera1',
                    'topic_name': 'camera1/image_raw',
                    'width': 1920,
                    'height': 1080,
                    'fps': 30
                }]
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])