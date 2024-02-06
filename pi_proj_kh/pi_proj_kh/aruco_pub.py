import cv2
import cv2.aruco as aruco
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

ids_before = 51

class ArucoDetectionNode(Node):
    def __init__(self):
        super().__init__('aruco_detection_node')
        self.cap = cv2.VideoCapture(0)
        self.qos_profile = QoSProfile(depth=10)
        self.publisher = self.create_publisher(String, 'Motor_control', self.qos_profile)
        self.timer = self.create_timer(0.1, self.detect_aruco_callback)

    def detect_aruco_callback(self):
        global ids_before
        ret, img = self.cap.read()

        if not ret:
            self.get_logger().warn('Failed to capture frame')
            time.sleep(1)
            return

        resized = cv2.resize(img, dsize=(640, 480), interpolation=cv2.INTER_LINEAR)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters_create()

        corners, ids, points = aruco.detectMarkers(resized, aruco_dict, parameters=parameters)

        if ids is not None:
            if ids_before != ids[0]:
                ids_before = ids[0]
                msg = String()
                msg.data = str(ids[0])
                self.publisher.publish(msg)
                self.get_logger().info(f'Detected ArUco Marker ID: {ids[0]}')

    def destroy(self):
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
