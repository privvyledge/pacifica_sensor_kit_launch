"""
Todo:
    * launch generic cameras using usb_node or v4l2_camera containers
    * launch azure kinect ros using container plugin or driver node
    * launch image rectification containers (e.g ROS or Nvidia)
    * launch Tensorrt_yolox containers for all cameras

"""

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch_ros.actions import Node
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.actions import OpaqueFunction
from launch.substitutions import EnvironmentVariable
from launch.substitutions import PathJoinSubstitution
import yaml


def launch_setup(context, *args, **kwargs):

    output_topic = LaunchConfiguration("output_topic").perform(context)

    camera_name = LaunchConfiguration("camera_name").perform(context)
    image_name = LaunchConfiguration("input_image").perform(context)
    image_rectification_topic = 'rgb/image_raw'  # image_raw, image_color
    camera_container_name = LaunchConfiguration("camera_container_name").perform(context)
    camera_namespace = camera_name + "/" + image_name
    input_camera_info = LaunchConfiguration("camera_info").perform(context)

    # tensorrt params
    yolo_image_topic = 'rgb/image_rect'  # image_rect, image_color, image_raw
    precision = LaunchConfiguration("precision").perform(context)
    data_path = PathJoinSubstitution([EnvironmentVariable('HOME'), 'autoware_data'])
    model_path = PathJoinSubstitution([data_path, 'tensorrt_yolox'])
    tensorrt_config_path = FindPackageShare('common_sensor_launch').perform(context) + "/config/perception/detection" + "/tensorrt_yolox" + ".param.yaml"

    with open(tensorrt_config_path, "r") as f:
        tensorrt_yaml_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    camera_param_path = FindPackageShare("common_sensor_launch").perform(context)+"/config/"+"monocam.param.yaml"
    with open(camera_param_path, "r") as f:
        camera_yaml_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    nodes = []

    camera_driver_node = Node(
                package="azure_kinect_ros_driver",
                executable='node',
                # name=camera_name + "azure_kinect_node",
                output='screen',
                parameters=[{
                    "color_enabled": LaunchConfiguration('color_enabled'),
                    "depth_enabled": LaunchConfiguration('depth_enabled'),
                    "color_resolution": LaunchConfiguration('color_resolution'),
                    "fps": LaunchConfiguration('fps'),
                    "depth_mode": LaunchConfiguration('depth_mode'),
                    "depth_unit": LaunchConfiguration('depth_unit'),
                    "color_format": LaunchConfiguration('color_format'),
                    "point_cloud": LaunchConfiguration('point_cloud'),
                    "rgb_point_cloud": LaunchConfiguration('rgb_point_cloud'),
                    "imu_rate_target": LaunchConfiguration('imu_rate_target'),
                    "point_cloud_in_depth_frame": LaunchConfiguration('point_cloud_in_depth_frame'),
                }],
                remappings=[
                    # ("imu", "azure_kinect/imu"),
                    # ("points2", "azure_kinect/pointcloud"),
                ],
            )

    image_debayer_node = ComposableNode(
                    package='image_proc',
                    plugin='image_proc::DebayerNode',
                    name=camera_name + '_debayer_node',
                    # Remap subscribers and publishers
                    remappings=[
                        ('image_raw', image_rectification_topic),  # input: ~/image_raw (camera_namespace + "/image_raw")
                        ('image_mono', 'image_mono'),  # output: ~/monochrome image
                        ('image_color', 'image_color'),  # output: ~/color_image
                    ],
            )

    color_image_rectification_node = ComposableNode(
                    # namespace="camera",
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name=camera_name + '_rectify_camera_image_node',
                    # Remap subscribers and publishers
                    remappings=[
                        ('image', image_rectification_topic),  # input (camera_namespace + "/image_raw")
                        ('camera_info', input_camera_info),
                        ('image_rect', 'image_rect')  # output
                    ],
                    extra_arguments=[
                        {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                    ],
            )

    monochrome_image_rectification_node = ComposableNode(
                    namespace="camera",
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name=camera_name + '_rectify_camera_monochrome_image_node',
                    # Remap subscribers and publishers
                    remappings=[
                        ('image', "image_mono"),  # input camera_namespace + "/image_mono"
                        ('camera_info', input_camera_info),
                        ('image_rect', 'image_rect_mono')  # output
                    ],
                    extra_arguments=[
                        {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                    ],
            )

    image_decompressor_node = ComposableNode(
                    package='image_transport_decompressor',
                    plugin='image_preprocessor::ImageTransportDecompressor',
                    name=camera_name + '_decompressor_node',
                    condition=IfCondition(LaunchConfiguration("use_decompress")),
                    # Remap subscribers and publishers
                    remappings=[
                        ('input/compressed_image', 'image_rect/compressed'),  # input: ~/image_raw (camera_namespace + "/image_raw")
                        ('output/raw_image', 'image_raw'),  # output: ~/monochrome image
                    ],
            )

    tensorrt_yolox_node = ComposableNode(
                namespace='/perception/object_recognition/detection',
                package="tensorrt_yolox",
                plugin="tensorrt_yolox::TrtYoloXNode",
                name=camera_name + "_tensorrt_yolox",
                parameters=[
                    {
                        "score_threshold": tensorrt_yaml_param['score_threshold'],
                        "nms_threshold": tensorrt_yaml_param['nms_threshold'],
                        "precision": precision,  # FP16, FP32, INT8
                        "data_path": data_path,
                        "model_path": model_path.perform(context) + LaunchConfiguration("model_name").perform(context) + ".onnx",
                        "label_path": model_path.perform(context) + "/label.txt",
                        "clip_value": tensorrt_yaml_param['clip_value'],
                        "preprocess_on_gpu": tensorrt_yaml_param['preprocess_on_gpu'],
                        "calibration_image_list_path": tensorrt_yaml_param['calibration_image_list_path'],
                        "build_only": tensorrt_yaml_param['build_only'],
                        "calibration_algorithm": tensorrt_yaml_param['calibration_algorithm'],
                        "dla_core_id": tensorrt_yaml_param['dla_core_id'],
                        "quantize_first_layer": tensorrt_yaml_param['quantize_first_layer'],
                        "quantize_last_layer": tensorrt_yaml_param['quantize_last_layer'],
                        "profile_per_layer": tensorrt_yaml_param['profile_per_layer'],
                    }
                ],
                remappings=[
                    ("in/image", yolo_image_topic),  # Input (camera_namespace + "/image_rect")
                    ("out/objects", output_topic),
                    ("out/image", output_topic + "/debug/image"),
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            )

    nodes.append(image_debayer_node)
    nodes.append(color_image_rectification_node)
    # nodes.append(monochrome_image_rectification_node)
    nodes.append(image_decompressor_node)
    nodes.append(tensorrt_yolox_node)

    container = ComposableNodeContainer(
        name=camera_container_name,
        namespace="/perception/object_detection",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        output="both",
        composable_node_descriptions=nodes,
        condition=UnlessCondition(LaunchConfiguration("use_camera_container")),
    )

    component_loader = LoadComposableNodes(
            composable_node_descriptions=nodes,
            target_container=camera_container_name,
            condition=IfCondition(LaunchConfiguration("use_camera_container")),
    )

    launch_data = [container, component_loader]
    launch_data.append(camera_driver_node)
    return launch_data


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        # a default_value of None is equivalent to not passing that kwarg at all
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    # Image arguments
    add_launch_arg("camera_name", "azure_kinect", description="camera name")
    add_launch_arg("color_enabled", "True", description="Enable or disable the color camera")
    add_launch_arg("depth_enabled", "True", description="Enable or disable the depth camera")
    add_launch_arg("color_resolution", "1440P",
                   description="Resolution at which to run the color camera. "
                               "Valid options: 720P, 1080P, 1440P, 1536P, 2160P, 3072P")
    add_launch_arg("fps", "30", description="FPS to run both cameras at. Valid options are 5, 15, and 30")
    add_launch_arg("depth_mode", "NFOV_UNBINNED",
                   description="Set the depth camera mode, which affects FOV, depth range, and camera resolution. "
                               "NFOV_UNBINNED, NFOV_2X2BINNED, WFOV_UNBINNED, WFOV_2X2BINNED, and PASSIVE_IR")
    add_launch_arg("depth_unit", "16UC1", description='Depth distance units. '
                                                      'Options are: "32FC1" (32 bit float metre) or '
                                                      '"16UC1" (16 bit integer millimetre)')
    add_launch_arg("color_format", "bgra", description="The format of RGB camera. Valid options: bgra, jpeg")
    add_launch_arg("point_cloud", "True", description="Generate a point cloud from depth data. Requires depth_enabled")
    add_launch_arg("rgb_point_cloud", "True", description="Colorize the point cloud using the RBG camera. "
                                                          "Requires color_enabled and depth_enabled")
    add_launch_arg("imu_rate_target", "0", description="Desired output rate of IMU messages. "
                                                       "Set to 0 (default) for full rate (1.6 kHz).")
    add_launch_arg("point_cloud_in_depth_frame", "False",
                   description="Whether the RGB pointcloud is rendered in the depth frame (true) or RGB frame (false). "
                               "Will either match the resolution of the depth camera (true) or the RGB camera (false).")
    add_launch_arg("input_image", "rgb/image_raw", description="input camera topic")
    add_launch_arg("input_image", "rgb/image_raw", description="input camera topic")
    add_launch_arg("camera_info", "rgb/camera_info", description="input camera info topic")
    add_launch_arg("use_decompress", "False", description="whether to decompress the raw image")

    # Yolo Arguments
    add_launch_arg("precision", "")
    add_launch_arg("output_topic", "", description="output YOLO")

    add_launch_arg("model_name", "", description="yolo model type")
    add_launch_arg("label_file", "", description="tensorrt node label file")

    # miscellaneous launch arguments
    add_launch_arg("camera_container_name", "")
    add_launch_arg("use_camera_container", "false")
    add_launch_arg("use_intra_process", "", "use intra process")
    add_launch_arg("use_multithread", "", "use multithread")

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
