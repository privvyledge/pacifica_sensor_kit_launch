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


# Livox ROS parameters
xfer_format = 0  # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic = 0  # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src = 0  # 0-lidar, others-Invalid data src
publish_freq = 50.0  # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type = 0
frame_id = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'


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
    Quanergy M8 configuration setup
    """
    package_directory = get_package_share_directory('common_sensor_launch')
    lidar_config = os.path.join(
            package_directory, 'config', 'lidar_livox_hap_config.json'
    )
    # publish_freq_la = DeclareLaunchArgument('publish_freq_la',
    #                                         default_value=str(publish_freq),
    #                                         description='LIDAR publish frequency. 5.0, 10.0, 20.0, 50.0, etc. Max 100.')
    #
    # lidar_frame_id_la = DeclareLaunchArgument('lidar_publish_frame_id',
    #                                           default_value=frame_id,
    #                                           description='Frame ID to publish pointclouds and IMU.')



    """
    Add PointCloud preprocessors
    """
    nodes = []
    # Box filter to remove the kart and other ego materials from the PointCloud
    glog_component_node = ComposableNode(
            package="glog_component",
            plugin="GlogComponent",
            name="glog_component",
        )

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
                ("input", "/quanergy/points"),
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
    # driver_component = ComposableNode(
    #     package="livox_ros_driver2",
    #     plugin="livox_ros::DriverNode",
    #     # node is created in a global context, need to avoid name clash
    #     name="livox_lidar_publisher",
    #     # parameters=[
    #     #     {
    #     #         **create_parameter_dict(
    #     #             "xfer_format",
    #     #              "multi_topic",
    #     #              "data_src",
    #     #              "publish_freq",
    #     #              "output_data_type",
    #     #              "frame_id",
    #     #              "lvx_file_path",
    #     #              "user_config_path",
    #     #              "cmdline_input_bd_code",
    #     #         ),
    #     #     }
    #     # ],
    #     parameters=livox_ros2_params,
    #     # remappings=[
    #     #     ("/livox/lidar", "pointcloud_raw_ex"),
    #     #     # ("/livox/points_ex", "pointcloud_raw_ex"),
    #     # ],
    #     # extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],  # todo: test this line
    # )

    # driver_node = Node(
    #     package='quanergy_client_ros',
    #     namespace=ns,
    #     executable='client_node',
    #     name='client_node',
    #     output='screen',
    #     arguments=[
    #             "--host", host,
    #             "--settings", PathJoinSubstitution(
    #                 [FindPackagePrefix('quanergy_client_ros'), 'settings', 'client.xml']),
    #             "--topic", topic,
    #             "--frame", frame
    #     ]
    # )

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
    # launch_data.append(driver_component_loader)
    # launch_data.append(driver_node)
    return launch_data


def generate_launch_description():
    host = LaunchConfiguration('host')
    ns = LaunchConfiguration('ns')
    topic = LaunchConfiguration('topic')
    frame = LaunchConfiguration('frame')

    package_directory = get_package_share_directory('common_sensor_launch')
    # lidar_config = os.path.join(
    #         package_directory, 'config', 'lidar_livox_hap_config.json'
    # )
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        # a default_value of None is equivalent to not passing that kwarg at all
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    add_launch_arg("host", description='Host name or IP of the sensor.')
    add_launch_arg("ns", default_value="quanergy", description='Namespace for the node.')
    add_launch_arg("topic", default_value="points", description='ROS topic for publishing the point cloud.')
    add_launch_arg("frame", default_value="quanergy", description='Frame name inserted in the point cloud')
    add_launch_arg("config_file", "", description="sensor configuration file")
    add_launch_arg("launch_driver", "True", "do launch driver")
    add_launch_arg("sensor_ip", "192.168.1.201", "device ip address")
    add_launch_arg("host_ip", "255.255.255.255", "host ip address")
    add_launch_arg("base_frame", "base_link", "base frame id")
    add_launch_arg("min_range", "0.3", "minimum view range for Quanergy sensors")
    add_launch_arg("max_range", "300.0", "maximum view range for Quanergy sensors")
    add_launch_arg("cloud_min_angle", "0", "minimum view angle setting on device")
    add_launch_arg("cloud_max_angle", "360", "maximum view angle setting on device")
    add_launch_arg("data_port", "2368", "device data port number")
    add_launch_arg("packet_mtu_size", "1500", "packet mtu size")
    add_launch_arg("rotation_speed", "600", "rotational frequency")
    add_launch_arg("dual_return_distance_threshold", "0.1", "dual return distance threshold")
    add_launch_arg("frame_id", "lidar", "frame id")
    add_launch_arg("input_frame", LaunchConfiguration("base_frame"), "use for cropbox")
    add_launch_arg("output_frame", LaunchConfiguration("base_frame"), "use for cropbox")
    add_launch_arg(
        "vehicle_mirror_param_file", description="path to the file of vehicle mirror position yaml"
    )
    add_launch_arg("use_multithread", "False", "use multithread")
    add_launch_arg("use_intra_process", "False", "use ROS 2 component container communication")
    add_launch_arg("use_pointcloud_container", "false")
    add_launch_arg("container_name", "livox_node_container")

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
