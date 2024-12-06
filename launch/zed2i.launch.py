import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.substitutions import PathJoinSubstitution as PJoin
from launch.substitutions import Command, TextSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    # Get the xacro file path
    xacro_path = LaunchConfig('xacro_file')

    # Get the common stereo configuration path
    stereo_config_file = LaunchConfig('stereo_config_file')
    config_common_path_val = stereo_config_file.perform(context)

    # Get the zed2i configuration path
    zed2i_config_file = LaunchConfig('zed2i_config_file')
    config_camera_path = zed2i_config_file.perform(context)

    # Get the ffmpeg configuration file
    config_ffmpeg = LaunchConfig('ffmpeg_config_file')

    # Get the local configuration file
    local_config_file = LaunchConfig('local_config_file')
    config_local_path = local_config_file.perform(context)
    with open(config_local_path, 'r') as f:
        config_local = yaml.safe_load(f)

    # Get the camera name
    camera_name_val = config_local['general']['camera_name']

    # Get the camera model
    camera_model_val = config_local['general']['camera_model']

    # Xacro command with options
    xacro_command = []
    xacro_command.append('xacro')
    xacro_command.append(' ')
    xacro_command.append(xacro_path.perform(context))
    xacro_command.append(' ')
    xacro_command.append('camera_name:=')
    xacro_command.append(camera_name_val)
    xacro_command.append(' ')
    xacro_command.append('camera_model:=')
    xacro_command.append(camera_model_val)

    # Robot State Publisher node
    rsp_node = Node(
        condition=IfCondition(TextSubstitution(
            text=str(config_local['pos_tracking']['publish_tf']).lower())),
        package='robot_state_publisher',
        namespace=camera_name_val,
        executable='robot_state_publisher',
        name='zed_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(xacro_command)
        }]
    )

    node_parameters = [
         
        # Default YAML files
        config_common_path_val,  # Common parameters
        config_camera_path,  # Camera related parameters
        config_ffmpeg, # FFMPEG parameters

        # Overriding YAML file
        config_local
    ]

    # Create the composabel node for the zed camera
    zed_wrapper_component = ComposableNode(
        package='zed_components',
        namespace=camera_name_val,
        plugin='stereolabs::ZedCamera',
        name='zed_node',
        parameters=node_parameters,
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # Create the composabel node container for the zed camera
    zed_container = ComposableNodeContainer(
            name='zed_container',
            namespace=camera_name_val,
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                zed_wrapper_component
            ],
            output='screen',
    )

    return [rsp_node, zed_container]

def generate_launch_description():
    return LaunchDescription(
        [
            LaunchArg(
                'xacro_file',
                default_value=PJoin([FindPackageShare('zed_wrapper'), 'urdf', 'zed_descr.urdf.xacro']),
                description='Path to the xacro file for the camera.',
            ),
            LaunchArg(
                'stereo_config_file',
                default_value=PJoin([FindPackageShare('zed_wrapper'), 'config', 'common_stereo.yaml']),
                description='Path to the default stereo camera YAML file.',
            ),
            LaunchArg(
                'zed2i_config_file',
                default_value=PJoin([FindPackageShare('zed_wrapper'), 'config', 'zed2i.yaml']),
                description='Path to the default zed2i camera YAML file.',
            ),
            LaunchArg(
                'ffmpeg_config_file',
                default_value=PJoin([FindPackageShare('zed_wrapper'), 'config', 'ffmpeg.yaml']),
                description='Path to the default ffmpeg camera YAML file.',
            ),
            LaunchArg(
                'local_config_file',
                default_value=PJoin([FindPackageShare('multi_cam_rig'), 'config', 'zed2i.yaml']),
                description='Path to the local camera YAML file (overrides defaults).',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
