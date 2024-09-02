import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32MultiArray
import pyrealsense2 as rs

import numpy as np
import math


class CameraPoseCirculate(Node):
    def __init__(self):
        super().__init__('imu_node')
        qos_profile = QoSProfile(depth=2)
        self.camera_pose = self.create_publisher(
            Float32MultiArray, 
            'imu_data', 
            qos_profile)
        
        
        ### parameters ###
        
        self.tilt_array = []
        self.sample_freq = 1/15
        
        ##################
        
        ### declare ###
        
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.accel)
        self.pipeline.start(self.config)
        
        ###############
        
        
        
        self.timer = self.create_timer(self.sample_freq, self.circulate_pose)
        
        
    ##left is minus,,,, right is plus,,,,
    def circulate_pose(self) :
        msg = Float32MultiArray()
        
        frames = self.pipeline.wait_for_frames()
        accel_frame = frames[0].as_motion_frame()
        
        if accel_frame:
            accel_datas = accel_frame.get_motion_data()
            accel_data = [accel_datas.x, accel_datas.y, accel_datas.z]
            
            norm = math.sqrt(accel_data[0]**2 + accel_data[1]**2 + accel_data[2]**2)
            accel_data = [np.arccos(accel_data[0]/norm) / np.pi * 180, np.arccos(- accel_data[1]/norm)/ np.pi * 180, np.arccos(accel_data[2]/norm) / np.pi * 180]
            
            # tilted_roll_angle = math.atan2(-accel_data[0], -accel_data[1]) /math.pi * 180
            
            # angle, x, y, z 
            msg.data = [accel_data[0],accel_data[1],accel_data[2]]
            # self.get_logger().info(f'{math.atan2(-msg.data[1], -msg.data[2]) / math.pi *180}')
            # self.get_logger().info(f'{accel_data[0]:.2f}  {accel_data[1]:.2f}  {accel_data[2]:.2f} ')
            
            self.camera_pose.publish(msg)
            
        
        

def main(args=None):
    rclpy.init(args=args)
    node = CameraPoseCirculate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()