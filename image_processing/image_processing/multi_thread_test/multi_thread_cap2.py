import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
import numpy as np
import pyrealsense2 as rs

import cv2
from cv_bridge import CvBridge
from threading import Thread, Event
import queue


class MultiThreadTestCap(Node):

    def __init__(self):
        super().__init__('multithread_test_capture')
            
        qos_profile = QoSProfile(depth=10)
        self.image_top_pub = self.create_publisher(Image, 'img_top', qos_profile)
        self.image_bot_pub = self.create_publisher(Image, 'img_bot', qos_profile)
        self.pub_timer = self.create_timer(1/30, self.publishs)

        ### Parameters ###
        self.img_size_x = 1280
        self.img_size_y = 720
        
        #############

        ### Initialize Variables ###
        self.img = np.zeros((self.img_size_y, self.img_size_x, 3), dtype=np.uint8)
        self.img_top = np.zeros((int(self.img_size_y/2), self.img_size_x, 3), dtype=np.uint8)
        self.img_bot = np.zeros((int(self.img_size_y/2), self.img_size_x, 3), dtype=np.uint8)

        self.cvbrid = CvBridge()
        self.frame_queue = queue.Queue(maxsize=1)
        self.stop_event = Event()
        
        self.capture_thread = Thread(target=self.image_capture_thread)
        self.capture_thread.start()

    def start_pipeline(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.config.enable_stream(rs.stream.color, self.img_size_x, self.img_size_y, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, self.img_size_x, self.img_size_y, rs.format.z16, 30)
        
        depth_profile = self.pipeline.start(self.config)
        depth_sensor = depth_profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        
        clipping_distance_in_meters = 1  # 1 meter
        self.clipping_distance = clipping_distance_in_meters / self.depth_scale
        
        align_to = rs.stream.color
        self.align = rs.align(align_to)
        self.get_logger().info("Pipeline started.")

    def image_capture_thread(self):
        self.start_pipeline()
        
        while not self.stop_event.is_set():
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            
            color_frame = aligned_frames.get_color_frame()
            aligned_depth_frame = aligned_frames.get_depth_frame()
            
            if not color_frame or not aligned_depth_frame:
                continue
            
            img = np.asanyarray(color_frame.get_data())
            img_top = img[:int(self.img_size_y/2), :]
            img_bot = img[int(self.img_size_y/2):, :]

            if not self.frame_queue.full():
                self.frame_queue.put((img, img_top, img_bot))
            
            self.get_logger().info("Frame captured.")

        self.pipeline.stop()

    def publishs(self):
        try:
            img, img_top, img_bot = self.frame_queue.get_nowait()
            self.img = img
            self.img_top = img_top
            self.img_bot = img_bot

            self.image_top_pub.publish(self.cvbrid.cv2_to_imgmsg(self.img_top))
            self.image_bot_pub.publish(self.cvbrid.cv2_to_imgmsg(self.img_bot))

            cv2.imshow("image capture", self.img)
            cv2.waitKey(1)
        except queue.Empty:
            pass

    def destroy_node(self):
        self.stop_event.set()
        self.capture_thread.join()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MultiThreadTestCap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
