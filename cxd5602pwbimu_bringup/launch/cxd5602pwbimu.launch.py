from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    rviz_config_path = get_package_share_directory('cxd5602pwbimu_bringup') + '/rviz/rvizconfig.rviz'
    cxp_node = Node(
        package='cxd5602pwbimu_node',
        executable='cxd5602pwbimu_node_exec',
        name='cxd5602pwbimu_node',
        output='screen',
        parameters=[{'device': '/dev/ttyUSB0'}]
    )

    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick_node',
        output='screen',
        parameters=[{'use_mag': False}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([cxp_node, imu_filter_node, rviz_node])



