import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pkg_share = FindPackageShare('multicam_calibration')

    rs_config_path = PathJoinSubstitution([pkg_share, 'config', 'realsense_calibration.yaml'])
    calib_config_path = PathJoinSubstitution([pkg_share, 'config', 'calibration_params.yaml'])

    # Read realsense config at generation time
    # FindPackageShare resolves after install, so we resolve it manually for generation
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    rs_config_file = os.path.join(pkg_dir, 'config', 'realsense_calibration.yaml')

    # Fallback to installed path if running from install space
    if not os.path.exists(rs_config_file):
        import ament_index_python
        pkg_dir = ament_index_python.get_package_share_directory('multicam_calibration')
        rs_config_file = os.path.join(pkg_dir, 'config', 'realsense_calibration.yaml')

    with open(rs_config_file, 'r') as f:
        rs_config = yaml.safe_load(f)

    calib_config_file = rs_config_file.replace(
        'realsense_calibration.yaml', 'calibration_params.yaml'
    )

    realsense_nodes = []
    for camera_cfg in rs_config['cameras']:
        node = ComposableNode(
            name=camera_cfg['camera_name'],
            namespace=camera_cfg['camera_name'],
            package='realsense2_camera',
            plugin='realsense2_camera::RealSenseNodeFactory',
            parameters=[rs_config['common_params'] | camera_cfg],
        )
        realsense_nodes.append(node)

    camera_container = ComposableNodeContainer(
        name='calibration_camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=realsense_nodes,
        output='screen',
    )

    calibration_node = Node(
        package='multicam_calibration',
        executable='calibration_node.py',
        name='multicam_calibration',
        output='screen',
        parameters=[{'config_file': calib_config_file}],
    )

    return LaunchDescription([
        camera_container,
        calibration_node,
    ])
