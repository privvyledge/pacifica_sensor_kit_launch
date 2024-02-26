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
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.actions import OpaqueFunction
from launch.substitutions import EnvironmentVariable
from launch.substitutions import PathJoinSubstitution
import yaml


def launch_setup(context, *args, **kwargs):

    output_topic = LaunchConfiguration("output_topic").perform(context)

    image_name = LaunchConfiguration("input_image").perform(context)
    camera_container_name = LaunchConfiguration("camera_container_name").perform(context)
    camera_namespace = LaunchConfiguration("camera_name").perform(context) + "/" + image_name
    input_camera_info = LaunchConfiguration("camera_info").perform(context)

    # tensorrt params
    precision = LaunchConfiguration("precision").perform(context)
    data_path = PathJoinSubstitution([EnvironmentVariable('USER', default_value='user'), 'autoware_data'])
    # data_path = PathJoinSubstitution(['/home', EnvironmentVariable('USER', default_value='user'), 'autoware_data'])
    model_path = PathJoinSubstitution([data_path, 'tensorrt_yolox'])
    tensorrt_config_path = FindPackageShare('common_sensor_launch').perform(context) + "/config/perception/detection" + "tensorrt_yolox" + ".param.yaml"

    with open(tensorrt_config_path, "r") as f:
        tensorrt_yaml_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    camera_param_path = FindPackageShare("common_sensor_launch").perform(context)+"/config/"+"monocam.param.yaml"
    with open(camera_param_path, "r") as f:
        camera_yaml_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    container = ComposableNodeContainer(
        name=camera_container_name,
        namespace="/perception/object_detection",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="usb_cam",
                plugin="usb_cam::UsbCamNode",
                name="usb_cam_node",
                parameters=[{
                    "camera_name": LaunchConfiguration('camera_name'),  # camera_yaml_param['camera_name']
                    "video_device": LaunchConfiguration('video_device'),  # camera_yaml_param['camera_name']
                    "framerate": LaunchConfiguration('framerate'),  # camera_yaml_param['frame_rate']
                    "io_method": camera_yaml_param['io_method'],
                    "frame_id": LaunchConfiguration('frame_id'),  # camera_yaml_param['frame_id']
                    "pixel_format": camera_yaml_param['pixel_format'],
                    # "serial_no": camera_yaml_param['serial_no'],
                    "camera_info_url": LaunchConfiguration('camera_info_url'),  # camera_yaml_param['camera_info_url']
                    "av_device_format": camera_yaml_param['av_device_format'],
                    "image_width": camera_yaml_param['image_width'],
                    "image_height": camera_yaml_param['image_height'],
                    "brightness": camera_yaml_param['brightness'],
                    "contrast": camera_yaml_param['contrast'],
                    "saturation": camera_yaml_param['saturation'],
                    "sharpness": camera_yaml_param['sharpness'],
                    "gain": camera_yaml_param['gain'],
                    "auto_white_balance": camera_yaml_param['auto_white_balance'],
                    "white_balance": camera_yaml_param['white_balance'],
                    "autoexposure": camera_yaml_param['autoexposure'],
                    "autofocus": camera_yaml_param['autofocus'],
                    "focus": camera_yaml_param['focus'],
                }],
                remappings=[
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            ),

            ComposableNode(
                    package='image_proc',
                    plugin='image_proc::DebayerNode',
                    name='debayer_node',
                    # Remap subscribers and publishers
                    remappings=[
                        ('image_raw', 'image_raw'),  # input: ~/image_raw (camera_namespace + "/image_raw")
                        ('image_mono', 'image_mono'),  # output: ~/monochrome image
                        ('image_color', 'image_color'),  # output: ~/color_image
                    ],
            ),

            ComposableNode(
                    namespace="camera",
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='rectify_camera_image_node',
                    # Remap subscribers and publishers
                    remappings=[
                        ('image', "image_raw"),  # input (camera_namespace + "/image_raw")
                        ('camera_info', input_camera_info),
                        ('image_rect', 'image_rect')  # output
                    ],
                    extra_arguments=[
                        {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                    ],
            ),

            # ComposableNode(
            #         namespace="camera",
            #         package='image_proc',
            #         plugin='image_proc::RectifyNode',
            #         name='rectify_camera_monochrome_image_node',
            #         # Remap subscribers and publishers
            #         remappings=[
            #             ('image', camera_namespace + "/image_mono"),  # input
            #             ('camera_info', input_camera_info),
            #             ('image_rect', 'image_rect_mono')  # output
            #         ],
            #         extra_arguments=[
            #             {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
            #         ],
            # ),

            # ComposableNode(
            #         package='image_transport_decompressor',
            #         plugin='image_preprocessor::ImageTransportDecompressor',
            #         name='decompressor_node',
            #         # Remap subscribers and publishers
            #         remappings=[
            #             ('input/compressed_image', 'image_raw/compressed'),  # input: ~/image_raw (camera_namespace + "/image_raw")
            #             ('output/raw_image', 'image_raw'),  # output: ~/monochrome image
            #         ],
            # ),

            ComposableNode(
                namespace='/perception/object_recognition/detection',
                package="tensorrt_yolox",
                plugin="tensorrt_yolox::TrtYoloXNode",
                name="tensorrt_yolox",
                parameters=[
                    {
                        "score_threshold": tensorrt_yaml_param['score_threshold'],
                        "nms_thresh": tensorrt_yaml_param['nms_thresh'],
                        "precision": precision,  # FP16, FP32, INT8
                        "data_path": data_path,
                        "model_path": model_path.perform(context) + LaunchConfiguration("model_name").onnx,
                        "label_path": model_path.perform(context) + LaunchConfiguration("label_file").onnx,
                        "clip_value": tensorrt_yaml_param['clip_value'],
                        "preprocess_on_gpu": tensorrt_yaml_param['preprocess_on_gpu'],
                        "calibration_image_list_path": tensorrt_yaml_param['calibration_image_list_path'],
                        "use_decompress": tensorrt_yaml_param['use_decompress'],
                        "build_only": tensorrt_yaml_param['build_only'],
                        "calibration_algorithm": tensorrt_yaml_param['calibration_algorithm'],
                        "dla_core_id": tensorrt_yaml_param['dla_core_id'],
                        "quantize_first_layer": tensorrt_yaml_param['quantize_first_layer'],
                        "quantize_last_layer": tensorrt_yaml_param['quantize_last_layer'],
                        "profile_per_layer": tensorrt_yaml_param['profile_per_layer'],
                    }
                ],
                remappings=[
                    ("in/image", "image_rect"),  # Input (camera_namespace + "/image_rect")
                    ("out/objects", output_topic),
                    ("out/image", output_topic + "/debug/image"),
                ],
                extra_arguments=[
                    {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
                ],
            ),
        ],

    )
    return [container]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        # a default_value of None is equivalent to not passing that kwarg at all
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    # Image arguments
    add_launch_arg("camera_name", "", description="camera name")
    add_launch_arg("video_device", "", description="video driver path")
    add_launch_arg("framerate", "", description="fps")
    add_launch_arg("frame_id", "", description="image_frame_id")
    add_launch_arg("input_image", "", description="input camera topic")
    add_launch_arg("camera_info", "", description="input camera info topic")
    add_launch_arg("camera_info_file", "", description="input camera info file")

    # Yolo Arguments
    add_launch_arg("precision", "")
    add_launch_arg("output_topic", "", description="output YOLO")

    add_launch_arg("model_name", "", description="yolo model type")
    add_launch_arg("label_file", "", description="tensorrt node label file")

    # miscellaneous launch arguments
    add_launch_arg("camera_container_name", "")
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
