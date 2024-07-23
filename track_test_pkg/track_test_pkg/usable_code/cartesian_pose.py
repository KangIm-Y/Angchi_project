import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import numpy as np

from std_msgs.msg import Float32MultiArray
from custom_interfaces.srv import PositionService

import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO

from cv_bridge import CvBridge

img_size_x = 1920
img_size_y = 1080
dimg_size_x = 1280
dimg_size_y = 720
HFOV = 69   # degree
VFOV = 42
DFOV = 77
focal_length = 0.00193  # m
pixel_size_color = 0.0000014  # m
pixel_size_depth = 0.000003  # m

class CartesianPoseNode(Node):
    def __init__(self):
        super().__init__('pub_for_testmodel')
        qos_profile = QoSProfile(depth=10)
        self.client = self.create_client(PositionService, 'object_coordinate')
        
        # while not self.client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Service not available, waiting again...')
        
        self.req = PositionService.Request()
        
        self.cvbrid = CvBridge()
        
        self.pipeline = rs.pipeline()
        config = rs.config()

        config.enable_stream(rs.stream.depth, dimg_size_x, dimg_size_y, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, img_size_x, img_size_y, rs.format.bgr8, 30)

        profile = self.pipeline.start(config)

        self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

        clipping_distance_in_meters = 1  # 1 meter
        clipping_distance = clipping_distance_in_meters / self.depth_scale
        align_to = rs.stream.color
        self.align = rs.align(align_to)

        self.coordinate = [0., 0., 0.]
        
        self.model = YOLO('yolov8n.pt')
        
        self.timer = self.create_timer(1/30, self.find_object)
        
    def find_object(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)

        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        self.depth_image = np.asanyarray(aligned_depth_frame.get_data())
        self.color_image = np.asanyarray(color_frame.get_data())
        
        result = self.model.predict(self.color_image, classes=[11], conf=0.5, max_det=1)
        annotated_img = result[0].plot()

        self.object_xy = 0
        if len(result[0].boxes.cls):
            self.object_xy = np.array(result[0].boxes.xywh.detach().numpy().tolist()[0], dtype='int')
            self.get_logger().info(f'{self.object_xy[0], self.object_xy[1]}')
            self.distance = self.depth_image[self.object_xy[1]][self.object_xy[0]] * self.depth_scale 
            annotated_img = cv2.circle(annotated_img, (self.object_xy[0], self.object_xy[1]), 10, (0, 0, 255), -1, cv2.LINE_AA)
            
            depth = aligned_depth_frame.get_distance(self.object_xy[0], self.object_xy[1])

            depth_intrinsics = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
            depth_point = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [self.object_xy[0], self.object_xy[1]], depth)
            self.get_logger().info(f"3D coordinates at pixel {self.object_xy}: {depth_point}")
            self.get_logger().info(f'X : {depth_point[2]}  Y : {-depth_point[0]}  Z : {-depth_point[1]}')
            
            self.coordinate = [depth_point[2], depth_point[0], depth_point[1]]
            
            self.send_request()
        else:
            self.get_logger().info('No object detected')

        cv2.imshow('yolo', annotated_img)
        cv2.waitKey(1)
        
    def send_request(self):
        self.req.coordinate = self.coordinate
        self.future = self.client.call_async(self.req)
        self.future.add_done_callback(self.handle_response)
        self.get_logger().info(f'Called {self.coordinate}')
        
    def handle_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Response is True, shutting down node.')
                rclpy.shutdown()
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CartesianPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
