import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

# Function to load a YAML file as a dictionary
def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, "r") as f:
        return yaml.safe_load(f)

# Function to create a Node for each camera configuration
def generate_camera_node(context, camera_name, params):
    return Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace=params.get("camera_namespace", camera_name),
        name=params.get("camera_name", camera_name),
        parameters=[params],
        output=params.get("output", "screen"),
        arguments=['--ros-args', '--log-level', params.get("log_level", "info")],
    )

# Function to set up camera nodes from YAML
def setup_cameras(context, config_file):
    # Resolve the LaunchConfiguration value
    config_file_path = config_file.perform(context)
    
    # Locate the `realsense2_camera` package
    realsense_package_path = get_package_share_directory("realsense2_camera")
    full_config_file_path = os.path.join(realsense_package_path, config_file_path)

    if not os.path.exists(full_config_file_path):
        raise FileNotFoundError(f"Configuration file '{full_config_file_path}' not found.")
    
    cameras_config = yaml_to_dict(full_config_file_path)

    camera_nodes = []
    for camera_name, params in cameras_config.items():
        camera_nodes.append(generate_camera_node(context, camera_name, params))
    
    return camera_nodes


def generate_launch_description():
    # Argument for YAML configuration file (relative to the `realsense2_camera` package)
    declare_config_file = DeclareLaunchArgument(
        "config_file",
        default_value="yaml/rs_launch_1.yaml",
        description="Path to the YAML configuration file relative to the realsense2_camera package",
    )

    # Launch cameras based on configuration file
    launch_cameras = OpaqueFunction(
        function=setup_cameras, kwargs={"config_file": LaunchConfiguration("config_file")}
    )

    return LaunchDescription([declare_config_file, launch_cameras])
