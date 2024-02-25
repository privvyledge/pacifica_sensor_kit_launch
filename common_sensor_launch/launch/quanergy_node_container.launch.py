"""
Todo:
    * pass configuration options as parameters
"""

import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackagePrefix
import yaml


def get_vehicle_info(context):
    # TODO(TIER IV): Use Parameter Substitution after we drop Galactic support
    # https://github.com/ros2/launch_ros/blob/master/launch_ros/launch_ros/substitutions/parameter.py
    gp = context.launch_configurations.get("ros_params", {})
    if not gp:
        gp = dict(context.launch_configurations.get("global_params", {}))
    p = {}
    p["vehicle_length"] = gp["front_overhang"] + gp["wheel_base"] + gp["rear_overhang"]
    p["vehicle_width"] = gp["wheel_tread"] + gp["left_overhang"] + gp["right_overhang"]
    p["min_longitudinal_offset"] = -gp["rear_overhang"]
    p["max_longitudinal_offset"] = gp["front_overhang"] + gp["wheel_base"]
    p["min_lateral_offset"] = -(gp["wheel_tread"] / 2.0 + gp["right_overhang"])
    p["max_lateral_offset"] = gp["wheel_tread"] / 2.0 + gp["left_overhang"]
    p["min_height_offset"] = 0.0
    p["max_height_offset"] = gp["vehicle_height"]
    return p


def get_vehicle_mirror_info(context):
    path = LaunchConfiguration("vehicle_mirror_param_file").perform(context)
    with open(path, "r") as f:
        p = yaml.safe_load(f)["/**"]["ros__parameters"]
    return p


def launch_setup(context, *args, **kwargs):
    def create_parameter_dict(*args):
        result = {}
        for x in args:
            result[x] = LaunchConfiguration(x)
        return result

    """
    Add PointCloud preprocessors
    """
    nodes = []

    glog_component_node = ComposableNode(
            package="glog_component",
            plugin="GlogComponent",
            name="glog_component",
        )

    # Box filter to remove the Van and other ego materials from the PointCloud
    cropbox_parameters = create_parameter_dict("input_frame", "output_frame")
    cropbox_parameters["negative"] = True

    vehicle_info = get_vehicle_info(context)
    cropbox_parameters["min_x"] = vehicle_info["min_longitudinal_offset"]
    cropbox_parameters["max_x"] = vehicle_info["max_longitudinal_offset"]
    cropbox_parameters["min_y"] = vehicle_info["min_lateral_offset"]
    cropbox_parameters["max_y"] = vehicle_info["max_lateral_offset"]
    cropbox_parameters["min_z"] = vehicle_info["min_height_offset"]
    cropbox_parameters["max_z"] = vehicle_info["max_height_offset"]

    ego_box_crop_node = ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::CropBoxFilterComponent",
            name="crop_box_filter_self",
            remappings=[
                ("input", "/quanergy/points_raw"),
                ("output", "self_cropped/pointcloud_ex"),
            ],
            parameters=[cropbox_parameters],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )

    mirror_info = get_vehicle_mirror_info(context)
    cropbox_parameters["min_x"] = mirror_info["min_longitudinal_offset"]
    cropbox_parameters["max_x"] = mirror_info["max_longitudinal_offset"]
    cropbox_parameters["min_y"] = mirror_info["min_lateral_offset"]
    cropbox_parameters["max_y"] = mirror_info["max_lateral_offset"]
    cropbox_parameters["min_z"] = mirror_info["min_height_offset"]
    cropbox_parameters["max_z"] = mirror_info["max_height_offset"]

    mirror_box_crop_node = ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::CropBoxFilterComponent",
            name="crop_box_filter_mirror",
            remappings=[
                ("input", "self_cropped/pointcloud_ex"),
                ("output", "mirror_cropped/pointcloud_ex"),
            ],
            parameters=[cropbox_parameters],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )

    pointcloud_interpolator_node = ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::DistortionCorrectorComponent",
            name="distortion_corrector_node",
            remappings=[
                ("~/input/twist", "/sensing/vehicle_velocity_converter/twist_with_covariance"),
                ("~/input/imu", "/sensing/imu/imu_data"),
                ("~/input/pointcloud", "mirror_cropped/pointcloud_ex"),
                ("~/output/pointcloud", "rectified/pointcloud_ex"),
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )

    ring_outlier_filter_node = ComposableNode(
            package="pointcloud_preprocessor",
            plugin="pointcloud_preprocessor::RingOutlierFilterComponent",
            name="ring_outlier_filter",
            remappings=[
                ("input", "rectified/pointcloud_ex"),
                ("output", "outlier_filtered/pointcloud"),
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )

    # todo: add more filters
    nodes.append(glog_component_node)
    nodes.append(ego_box_crop_node)
    nodes.append(mirror_box_crop_node)  # todo: might remove
    nodes.append(pointcloud_interpolator_node)  # todo: might remove
    nodes.append(ring_outlier_filter_node)  # todo: might remove

    # set container to run all required components in the same process
    container = ComposableNodeContainer(
        name=LaunchConfiguration("container_name"),
        namespace="pointcloud_preprocessor",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=nodes,
        condition=UnlessCondition(LaunchConfiguration("use_pointcloud_container")),
        output="both",
    )

    component_loader = LoadComposableNodes(
        composable_node_descriptions=nodes,
        target_container=LaunchConfiguration("container_name"),
        condition=IfCondition(LaunchConfiguration("use_pointcloud_container")),
    )

    # Start the LIDAR driver. Launch in parent XML launch
    driver_node = Node(
        package='quanergy_client_ros',
        namespace=LaunchConfiguration('ns'),
        executable='client_node',
        condition=IfCondition(LaunchConfiguration("launch_driver")),
        name='client_node',
        output='screen',
        arguments=[
                "--host", LaunchConfiguration('sensor_ip'),
                "--settings", LaunchConfiguration('config_file'),
                # "--use-ros-time", use_ros_time,
                "--topic", LaunchConfiguration('topic'),
                "--frame", LaunchConfiguration("frame_id"),
                # "--calibrate", LaunchConfiguration("calibrate"),
                "--frame-rate", LaunchConfiguration("frame_rate"),
                "--return", LaunchConfiguration("return_type")
        ]
    )

    target_container = (
        container
        if UnlessCondition(LaunchConfiguration("use_pointcloud_container")).evaluate(context)
        else LaunchConfiguration("container_name")
    )

    # driver_component_loader = LoadComposableNodes(
    #     composable_node_descriptions=[driver_component],
    #     target_container=target_container,
    #     condition=IfCondition(LaunchConfiguration("launch_driver")),
    # )

    launch_data = [container, component_loader]
    launch_data.append(target_container)
    launch_data.append(driver_node)
    return launch_data


def generate_launch_description():

    package_directory = get_package_share_directory('common_sensor_launch')
    lidar_config = PathJoinSubstitution(
                    [FindPackagePrefix('common_sensor_launch'), 'config', 'client.xml'])
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        # a default_value of None is equivalent to not passing that kwarg at all
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    add_launch_arg("sensor_ip", "192.168.10.166", "Host name or IP of the sensor.")
    add_launch_arg("ns", default_value="quanergy", description='Namespace for the node.')
    add_launch_arg("topic", default_value="points", description='ROS topic for publishing the point cloud.')
    add_launch_arg("frame", default_value="quanergy", description='Frame name inserted in the point cloud')
    add_launch_arg("return_type", default_value="all", description='0, 1, 2, all, all_separate_topics')
    add_launch_arg("calibrate", default_value="False",
                   description='whether to calculate the parameters from sensor data before applying')
    add_launch_arg("frame_rate", default_value="15.0",
                   description='frame rate of the sensor; used when calibrate == true only')
    add_launch_arg("use_ros_time", default_value="True",
                   description='Flag determining whether to use ROS time in message. Uses sensor time otherwise')
    add_launch_arg("config_file", lidar_config, description="sensor configuration file")
    add_launch_arg("launch_driver", "True", "do launch driver")
    add_launch_arg("base_frame", "base_link", "base frame id")
    add_launch_arg("frame_id", "quanergy", "frame id")
    add_launch_arg("input_frame", LaunchConfiguration("base_frame"), "use for cropbox")
    add_launch_arg("output_frame", LaunchConfiguration("base_frame"), "use for cropbox")
    add_launch_arg(
        "vehicle_mirror_param_file", description="path to the file of vehicle mirror position yaml"
    )
    add_launch_arg("use_multithread", "False", "use multithread")
    add_launch_arg("use_intra_process", "False", "use ROS 2 component container communication")
    add_launch_arg("use_pointcloud_container", "false")
    add_launch_arg("container_name", "quanergy_node_container")

    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
        condition=UnlessCondition(LaunchConfiguration("use_multithread")),
    )

    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
        condition=IfCondition(LaunchConfiguration("use_multithread")),
    )

    return launch.LaunchDescription(
        launch_arguments
        + [set_container_executable, set_container_mt_executable]
        + [OpaqueFunction(function=launch_setup)]
    )
