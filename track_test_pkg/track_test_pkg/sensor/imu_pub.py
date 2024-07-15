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
        qos_profile = QoSProfile(depth=10)
        self.camera_pose = self.create_publisher(
            Float32MultiArray, 
            'imu_data', 
            qos_profile)
        
        
        ### parameters ###
        
        self.tilt_array = []
        self.time_period = 0.05
        
        ##################
        
        ### declare ###
        
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.accel)
        self.pipeline.start(self.config)
        
        ###############
        
        ### initialize ###
        
        frames = self.pipeline.wait_for_frames()
        accel_frame = frames[0].as_motion_frame()
        if accel_frame:
            accel_data = accel_frame.get_motion_data()
        
        for i in range(5) :
            self.tilt_array.append([accel_data.x,accel_data.y,accel_data.z])
        
        ##################
        
        
        self.timer = self.create_timer(self.time_period, self.circulate_pose)
        
        
    ##left is minus,,,, right is plus,,,,
    def circulate_pose(self) :
        msg = Float32MultiArray()
        
        frames = self.pipeline.wait_for_frames()
        accel_frame = frames[0].as_motion_frame()
        
        if accel_frame:
            accel_data = accel_frame.get_motion_data()
            
            self.tilt_array.pop(0)
            self.tilt_array.append([accel_data.x,accel_data.y,accel_data.z])
            
            avg_tilt_array = np.mean(self.tilt_array, axis=0)
            tilted_roll_angle = math.atan2(-avg_tilt_array[0], -avg_tilt_array[1]) /math.pi * 180
            
            # angle, x, y, z 
            msg.data = [tilted_roll_angle, avg_tilt_array[0],avg_tilt_array[1],avg_tilt_array[2]]
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