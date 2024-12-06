import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ns = '/multi_cam_rig'
    director_topic = os.path.join(ns, 'director')
    zed_left_output_topic = os.path.join(ns, 'zed_left_captured_image')
    zed_right_output_topic = os.path.join(ns, 'zed_right_captured_image')
    firefly_left_output_topic = os.path.join(ns, 'firefly_left_captured_image')
    firefly_right_output_topic = os.path.join(ns, 'firefly_right_captured_image')
    firefly_left_input_topic = '/flir_node/firefly_left/image_raw'
    firefly_right_input_topic = '/flir_node/firefly_right/image_raw'
    serial_port = '/dev/ttyUSB0'

    trigger_gui_node = Node(
        package='multi_cam_rig',
        executable='trigger_gui_node',
        name='trigger_gui_node',
        output='screen',
        parameters=[
            {'director_topic': director_topic}
        ]
    )

    zed_image_node = Node(
        package='multi_cam_rig',
        executable='zed_image_node',
        name='zed_image_node',
        output='screen',
        parameters=[
            {'director_topic': director_topic},
            {'left_output_topic': zed_left_output_topic},
            {'right_output_topic': zed_right_output_topic}            
        ]
    )

    firefly_image_node = Node(
        package='multi_cam_rig',
        executable='firefly_image_node',
        name='firefly_image_node',
        output='screen',
        parameters=[
            {'director_topic': director_topic},
            {'left_output_topic': firefly_left_output_topic},
            {'right_output_topic': firefly_right_output_topic},
            {'left_input_topic': firefly_left_input_topic},
            {'right_input_topic': firefly_right_input_topic},
            {'serial_port': serial_port}
        ]
    )

    return LaunchDescription([
        trigger_gui_node, zed_image_node, firefly_image_node
    ])
