import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
import serial

global header1
global header2
global id
global dataSize
global mode
poseArr = []


header1 = 0xFF
header2 = 0xFE
id = 0x00
dataSize = 0x02
mode = 0x00
PI = 3.141592653589793


class JointStateSubscriber(Node):

    def __init__(self):
        super().__init__('JointState_subscriber')
        qos_profile = QoSProfile(depth=10)
        self.jointstate_subscriber = self.create_subscription(
            JointState,
            'joint_states',
            self.subscribe_topic_message,
            qos_profile)

    def subscribe_topic_message(self, msg):
        poseArr = msg.position
        for i in range(0,len(poseArr)):
            print("joint",i,poseArr[i])
            poseArr[i] = round((180 / PI) * poseArr[i])

        for i in range(0,len(poseArr)):
            print("joint",i,poseArr[i])



def main(args=None):
    rclpy.init(args=args)
    node = JointStateSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
