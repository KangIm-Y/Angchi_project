import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import String

import cv2
from cv_bridge import CvBridge


motor_control = String()

class WebcamSubscriber(Node) :
	def __init__(self) :
		super().__init__('webcam_Subscriber')
		qos = QoSProfile(depth=10)
		self.webcam_sub = self.create_subscription(Image, 'webcam', self.webcam_callback, qos)
		self.motor_pub = self.create_publisher(String, '/Motor_control', qos)
		self.aruco_pub = self.create_publisher(Image, '/aruco', qos)
		self.bridge = CvBridge() 
		   


	def webcam_callback(self, msg) :
		image = self.bridge.imgmsg_to_cv2(msg)
		#self.detect_aruco() = aruco_frame, aruco_ids
		aruco_frame, aruco_ids = self.detect_aruco(image)
		
		if aruco_ids == 1 :
			motor_control.data = 'Left'
			
		elif aruco_ids == 2 :
			motor_control.data = 'Right'
			
		else :
			motor_control.data = 'Retry'
		
		self.motor_pub.publish(motor_control)
		   
     
	def detect_aruco(self, frame):
		aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
		aruco_param = cv2.aruco.DetectorParameters_create()
		corners, ids, rejected = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_param)
		detect_markers = self.aruco_display(corners, ids, rejected, frame)
		return detect_markers, ids
    
	def aruco_display(self, corners, ids, rejected, frame):
		if len(corners) > 0:
			ids = ids.flatten()
			for (markerCorner, markerID) in zip(corners, ids):
				corners = markerCorner.reshape((4, 2))
				(topLeft, topRight, bottomRight, bottomLeft) = corners
				topRight = (int(topRight[0]), int(topRight[1]))
				bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
				bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
				topLeft = (int(topLeft[0]), int(topLeft[1]))

				cv2.line(frame, topLeft, topRight, (0, 255, 0), 1)
				cv2.line(frame, topRight, bottomRight, (0, 255, 0), 1)
				cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 1)
				cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 1)

				cX = int((topLeft[0] + bottomRight[0]) / 2.0)
				cY = int((topLeft[1] + bottomRight[1]) / 2.0)

				cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)

				cv2.putText(frame, str(markerID), (topLeft[0], topLeft[1]+15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
		return frame
	     
def main(args=None) :
  rclpy.init(args=args)
  node = WebcamSubscriber()

  try :
    rclpy.spin(node)
  except KeyboardInterrupt :
    node.get_logger().info('Stopped by Keyboard')
  finally :
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__' :
  main()
