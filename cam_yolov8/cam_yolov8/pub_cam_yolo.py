import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Int32, String

class CamYoloPublisher(Node):

    def __init__(self):
        super().__init__('pub_cam_yolo')
        qos_profile = QoSProfile(depth=10)
        self.cam_yolo_pub = self.create_publisher(String, 'Gesture', qos_profile)
        self.timer = self.create_timer(1, self.publish_msg)
        self.count = 0

    def publish_msg(self):
        send_data = String()
        send_data.data = "Left"
        self.cam_yolo_pub.publish(send_data)
        
def main(args=None):
    rclpy.init(args=args)
    node = CamYoloPublisher()
    try:
        while rclpy.ok():
            node.publish_msg()

    except KeyboardInterrupt:
        node.get_logger().info('Node Exit :: Keyboard Interrupt')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()