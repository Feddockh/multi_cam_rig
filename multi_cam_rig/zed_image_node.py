import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import pyzed.sl as sl

class ZedImageNode(Node):
    def __init__(self):
        super().__init__('zed_image_node')

        # Declare parameters
        self.declare_parameter('director_topic', '/multi_cam_rig/director')
        self.declare_parameter('left_output_topic', '/multi_cam_rig/zed_left_captured_image')
        self.declare_parameter('right_output_topic', '/multi_cam_rig/zed_right_captured_image')

        # Get parameters
        self.director_topic = self.get_parameter('director_topic').value
        self.left_output_topic = self.get_parameter('left_output_topic').value
        self.right_output_topic = self.get_parameter('right_output_topic').value

        # Publisher and subscriber for director topic
        self.director_publisher = self.create_publisher(String, self.director_topic, 10)
        self.director_subscriber = self.create_subscription(
            String,
            self.director_topic,
            self.director_callback,
            10
        )

        # Publisher for output image topics
        self.left_output_publisher = self.create_publisher(Image, self.left_output_topic, 10)
        self.right_output_publisher = self.create_publisher(Image, self.right_output_topic, 10)

        # Initialize ZED camera
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.camera_fps = 60
        self.zed.open(init_params)

        # Create a CvBridge object
        self.bridge = CvBridge()

    def director_callback(self, msg):
        if msg.data.startswith('img'):

            # Capture the images in side by side format
            capture = self.capture_images()

            # Respond on the director topic
            # We are responding before processing the images to avoid blocking the imaging sequence
            if capture is not None:
                response = String()
                response.data = 'zed complete'
                self.director_publisher.publish(response)
                self.get_logger().info('ZED image captured.')
            else:
                response = String()
                response.data = 'zed failed'
                self.director_publisher.publish(response)
                self.get_logger().warn('Failed to capture ZED image.')

            # Format images
            left_image, right_image = self.format_images(capture)

            # Publish the images
            self.left_output_publisher.publish(left_image)
            self.right_output_publisher.publish(right_image)

    def capture_images(self):
        runtime_parameters = sl.RuntimeParameters()
        if self.zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            side_by_side_image = sl.Mat()
            self.zed.retrieve_image(side_by_side_image, sl.VIEW.SIDE_BY_SIDE)
            return side_by_side_image
        else:
            self.get_logger().warn('Failed to capture images.')
            return None
        
    def format_images(self, side_by_side_image):
        
        # Convert image to OpenCV format
        side_by_side_image_ocv = side_by_side_image.get_data()

        # Split the side-by-side image into left and right images
        _, width, _ = side_by_side_image_ocv.shape
        left_image_ocv = side_by_side_image_ocv[:, :width // 2]
        right_image_ocv = side_by_side_image_ocv[:, width // 2:]

        # Convert BGRA to RGB
        left_image_rgb = cv2.cvtColor(left_image_ocv, cv2.COLOR_BGRA2RGB)
        right_image_rgb = cv2.cvtColor(right_image_ocv, cv2.COLOR_BGRA2RGB)

        # Convert images to ROS Image messages
        left_image_msg = self.bridge.cv2_to_imgmsg(left_image_rgb, encoding="rgb8")
        right_image_msg = self.bridge.cv2_to_imgmsg(right_image_rgb, encoding="rgb8")

        return left_image_msg, right_image_msg

    def destroy_node(self):
        self.zed.close()
        self.get_logger().info('ZED camera closed.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ZedImageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

























# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# from sensor_msgs.msg import Image

# class ZedImageNode(Node):
#     def __init__(self):
#         super().__init__('zed_image_node')

#         # Declare parameters
#         self.declare_parameter('director_topic', '/multi_cam_rig/director')
#         self.declare_parameter('left_input_topic', '/zed/left/image_rect_color')
#         self.declare_parameter('right_input_topic', '/zed/right/image_rect_color')
#         self.declare_parameter('left_output_topic', '/multi_cam_rig/zed_left_captured_image')
#         self.declare_parameter('right_output_topic', '/multi_cam_rig/zed_right_captured_image')

#         # Get parameters
#         self.director_topic = self.get_parameter('director_topic').value
#         self.left_input_topic = self.get_parameter('left_input_topic').value
#         self.right_input_topic = self.get_parameter('right_input_topic').value
#         self.left_output_topic = self.get_parameter('left_output_topic').value
#         self.right_output_topic = self.get_parameter('right_output_topic').value

#         # Publisher and subscriber for director topic
#         self.director_publisher = self.create_publisher(String, self.director_topic, 10)
#         self.director_subscriber = self.create_subscription(
#             String,
#             self.director_topic,
#             self.director_callback,
#             10
#         )

#         # Publisher for output image topics
#         self.left_output_publisher = self.create_publisher(Image, self.left_output_topic, 10)
#         self.right_output_publisher = self.create_publisher(Image, self.right_output_topic, 10)
        
#         # Subscriber for input image topics
#         self.left_input_subscriber = self.create_subscription(
#             Image,
#             self.left_input_topic,
#             self.left_image_callback,
#             10
#         )
#         self.right_input_subscriber = self.create_subscription(
#             Image,
#             self.right_input_topic,
#             self.right_image_callback,
#             10
#         )

#         self.latest_left_image = None
#         self.latest_right_image = None

#     def director_callback(self, msg):
#         if msg.data.startswith('image'):
#             if self.latest_left_image and self.latest_right_image:

#                 # Publish the images
#                 self.left_output_publisher.publish(self.latest_left_image)
#                 self.right_output_publisher.publish(self.latest_right_image)

#                 # Respond on the director topic
#                 response = String()
#                 response.data = 'zed complete'
#                 self.director_publisher.publish(response)

#             else:
#                 self.get_logger().warn('No image received yet.')

#     def left_image_callback(self, msg):
#         self.latest_left_image = msg

#     def right_image_callback(self, msg):
#         self.latest_right_image = msg

# def main(args=None):
#     rclpy.init(args=args)
#     node = ZedImageNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()