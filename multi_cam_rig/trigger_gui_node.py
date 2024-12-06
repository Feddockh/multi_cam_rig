import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk

class TriggerGuiNode(Node):
    def __init__(self):
        super().__init__('trigger_gui_node')

        # Declare and get parameters
        self.declare_parameter('director_topic', '/multi_cam_rig/director')
        self.director_topic = self.get_parameter('director_topic').value

        # Publisher
        self.director_publisher = self.create_publisher(String, self.director_topic, 10)

        # Initialize GUI
        self.root = tk.Tk()
        self.root.title('Trigger GUI')

        # Button to capture image
        self.button = tk.Button(self.root, text='Capture Image', command=self.on_button_press)
        self.button.pack(padx=20, pady=20)

        # Start the GUI loop in a separate thread
        self.timer = self.create_timer(0.1, self.gui_loop)

        self.image_count = 1

    def on_button_press(self):
        msg = String()
        msg.data = f'img {self.image_count}'
        self.director_publisher.publish(msg)
        self.get_logger().info(f'Sent: {msg.data}')
        self.image_count += 1

    def gui_loop(self):
        self.root.update_idletasks()
        self.root.update()

def main(args=None):
    rclpy.init(args=args)
    node = TriggerGuiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()