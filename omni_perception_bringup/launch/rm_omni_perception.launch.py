from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('omni_perception_bringup')
    
    # 配置文件路径
    perception_config = os.path.join(pkg_share, 'config', 'omni_perception_params.yaml')
    camera_config = os.path.join(pkg_share, 'config', 'camera_params.yaml')
    
    # 创建相机节点
    camera_left_node = Node(
        package='usb_camera',
        executable='usb_camera_node',
        name='camera_left',
        parameters=[camera_config],
        # namespace='camera_left',
        output='screen'
    )

    camera_mid_node = Node(
        package='usb_camera',
        executable='usb_camera_node',
        name='camera_mid',
        parameters=[camera_config],
        # namespace='camera_mid',
        output='screen'
    )

    camera_right_node = Node(
        package='usb_camera',
        executable='usb_camera_node',
        name='camera_right',
        parameters=[camera_config],
        # namespace='camera_right',
        output='screen'
    )

    # 创建感知节点
    omni_perception_node = Node(
        package='rm_omni_perception',
        executable='omni_perception_node',
        name='omni_perception_node',
        output='screen',
        parameters=[perception_config],
        remappings=[
            ('/image_raw', '/camera1/image_raw')  # 修改重映射的话题名称
        ]
    )

    # 创建延迟启动的感知节点
    delayed_omni_node = TimerAction(
        period=2.0,  # 延迟2秒
        actions=[omni_perception_node]
    )

    return LaunchDescription([
        camera_left_node,
        camera_mid_node,
        camera_right_node,
        delayed_omni_node
    ]) 