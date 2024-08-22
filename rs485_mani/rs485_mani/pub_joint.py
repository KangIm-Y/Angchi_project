import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Int32MultiArray


class JointPublisher(Node):

    def __init__(self):
        super().__init__('joint_publisher')
        qos_profile = QoSProfile(depth=10)
        self.joint_publisher = self.create_publisher(Int32MultiArray, 'joint', qos_profile)
        self.timer = self.create_timer(1, self.publish_joint_msg)
        self.count = 0

    def publish_joint_msg(self):
        msg = Int32MultiArray()
        msg.data = []
        for i in range(4):
            self.get_logger().info(f'[{self.count}] , {i} 번째 각도 : ')
            try : 
                deg = int(input())
                msg.data.append(deg)
            except :
                msg.data = [0,0,0,0]
                break

        self.joint_publisher.publish(msg)
        self.get_logger().info(f'[{self.count}] Published message: {msg.data}')
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = JointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
