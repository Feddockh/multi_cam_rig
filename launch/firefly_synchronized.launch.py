from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.substitutions import PathJoinSubstitution as PJoin
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
import yaml


def launch_setup(context, *args, **kwargs):

    # Get the path to the configuration file
    config_file = LaunchConfig('config_file').perform(context)

    # Get the path to the parameter and calibration directories
    parameter_directory = LaunchConfig('parameter_directory').perform(context)
    calibration_directory = LaunchConfig('calibration_directory').perform(context)

    # Load the configuration file
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)

    # Get the names of the cameras
    camera_names = config['cameras']

    # Replace placeholder for the parameter file
    config[camera_names[0]]['parameter_file'] = parameter_directory + '/firefly.yaml'
    config[camera_names[1]]['parameter_file'] = parameter_directory + '/firefly.yaml'

    # Get the serial numbers of the cameras
    serial_numbers = [config[camera_name]['serial_number'] for camera_name in camera_names]

    # Replace placeholders for the calibration files
    config[camera_names[0]]['camerainfo_url'] = calibration_directory + '/' + serial_numbers[0] + '.yaml'
    config[camera_names[1]]['camerainfo_url'] = calibration_directory + '/' + serial_numbers[1] + '.yaml'

    # Create the composabel node container for the synchronized camera driver
    container = ComposableNodeContainer(
        name='flir_node_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='spinnaker_synchronized_camera_driver',
                plugin='spinnaker_synchronized_camera_driver::SynchronizedCameraDriver',
                name='flir_node',
                parameters=[config],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )
    return [container]

def generate_launch_description():
    return LaunchDescription(
        [
            LaunchArg(
                'config_file',
                default_value=PJoin([FindPackageShare('multi_cam_rig'), 'config', 'firefly.yaml']),
                description='Path to the camera parameters YAML file.',
            ),
            LaunchArg(
                'parameter_directory',
                default_value=PJoin([FindPackageShare('spinnaker_camera_driver'), 'config']),
                description='root directory for camera parameter definitions',
            ),
            LaunchArg(
                'calibration_directory',
                default_value=PJoin([FindPackageShare('multi_cam_rig'), 'calibration']),
                description='root directory for camera calibration files',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
