import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String

from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class ArmyDetectionIndi(Node):

    def __init__(self):
        super().__init__('army_detection_indi')
        qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10  # depth 설정은 필요에 따라 조정
        )
        self.image_subscribtion = self.create_subscription(
            Image,
            'side_camera',
            self.sub_callback,
            qos_profile)
        self.cvbrid = CvBridge()

    def sub_callback(self, msg):
        current_img = self.cvbrid.imgmsg_to_cv2(msg)
        y,x,c = current_img.shape
        resized = cv2.resize(current_img, (int(x*1.5),int(y*1.5)), interpolation=cv2.INTER_CUBIC)

        cv2.imshow("side_camera", resized)
        cv2.waitKey(1)



def main(args=None):
    rclpy.init(args=args)
    node = ArmyDetectionIndi()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()