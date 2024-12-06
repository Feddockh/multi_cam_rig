import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import serial

class FireflyImageNode(Node):
    def __init__(self):
        super().__init__('firefly_image_node')

        # Declare parameters
        self.declare_parameter('director_topic', '/multi_cam_rig/director')
        self.declare_parameter('left_output_topic', '/multi_cam_rig/firefly_left_captured_image')
        self.declare_parameter('right_output_topic', '/multi_cam_rig/firefly_right_captured_image')
        self.declare_parameter('left_input_topic', '/flir_node/firefly_left/image_raw')
        self.declare_parameter('right_input_topic', '/flir_node/firefly_right/image_raw')
        self.declare_parameter('serial_port', '/dev/ttyUSB0')

        # Get parameters
        self.director_topic = self.get_parameter('director_topic').value
        self.left_output_topic = self.get_parameter('left_output_topic').value
        self.right_output_topic = self.get_parameter('right_output_topic').value
        self.left_input_topic = self.get_parameter('left_input_topic').value
        self.right_input_topic = self.get_parameter('right_input_topic').value
        self.serial_port = self.get_parameter('serial_port').value

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

        # Subscriber for input image topics
        self.left_input_subscriber = self.create_subscription(
            Image,
            self.left_input_topic,
            self.left_image_callback,
            10
        )
        self.right_input_subscriber = self.create_subscription(
            Image,
            self.right_input_topic,
            self.right_image_callback,
            10
        )

        # Initialize serial communication
        try:
            self.serial_connection = serial.Serial(self.serial_port, baudrate=9600, timeout=1)
            self.get_logger().info(f'Connected to serial port {self.serial_port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to serial port: {e}')
            self.serial_connection = None

        self.left_image = None
        self.right_image = None

    def director_callback(self, msg):
        if msg.data.startswith('zed complete'):

            # Send the serial trigger message
            if self.serial_connection:
                self.serial_connection.write(b't')
                self.get_logger().info('Sent serial trigger message.')
            else:
                self.get_logger().warn('Serial connection not established.')

    def left_image_callback(self, msg):

        # Store the image
        if self.left_image is None:
            self.left_image = msg

        # Check if both images are available
        if self.right_image is not None:

            # Send message to director
            response = String()
            response.data = 'firefly complete'
            self.director_publisher.publish(response)
            self.get_logger().info('Sent firefly complete message.')

            # Publish the images
            self.publish_images()

    def right_image_callback(self, msg):

        # Store the image
        if self.right_image is None:
            self.right_image = msg

        # Check if both images are available
        if self.left_image is not None:

            # Send message to director
            response = String()
            response.data = 'firefly complete'
            self.director_publisher.publish(response)
            self.get_logger().info('Sent firefly complete message.')

            # Publish the images
            self.publish_images()

    def publish_images(self):
        self.left_output_publisher.publish(self.left_image)
        self.right_output_publisher.publish(self.right_image)
        self.left_image = None
        self.right_image = None

    def destroy_node(self):
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FireflyImageNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()