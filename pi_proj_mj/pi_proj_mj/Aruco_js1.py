import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()

class camPublisher(Node) :
  def __init__(self) :
    super().__init__('WebCam_Publisher')
    self.publisher = self.create_publisher(Image, 'webcam', 10)
    time_period = 0.01
    self.timer = self.create_timer(time_period, self.time_callback)
    self.cap = cv2.VideoCapture(4) 
    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
 

  def time_callback(self) :
    ret, frame = self.cap.read()
 
    if ret == True :
      fra = bridge.cv2_to_imgmsg(frame)
      self.publisher.publish(fra)
      cv2.imshow('webcam_raw', frame)
      cv2.waitKey(2)
    #self.get_logger().info('Publishing Raw Image')


def main(args=None) :
  rclpy.init(args=args)
  node = camPublisher()
  try :
    rclpy.spin(node)
  except KeyboardInterrupt :
    node.get_logger().info('Publish Stopped')
  finally :
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__' :
  main()
