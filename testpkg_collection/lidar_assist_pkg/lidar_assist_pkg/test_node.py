import rclpy
from rclpy.qos import QoSProfile
from std_msgs.msg import String
import serial
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from rclpy.parameter import Parameter
from time import sleep
import time
import copy
import math
from geometry_msgs.msg import Twist, Pose, Point, Vector3, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from tf2_ros import TransformBroadcaster

class OdomTestNode(Node):
    def __init__(self):
        super().__init__('odom_test_node')
        self.pub_OdomTF = TransformBroadcaster(self)
        self.pub_Odom = self.create_publisher(Odometry, 'odom', 10)
        
        self.odom_pose = OdomPose()
        self.odom_pose.timestamp = self.get_clock().now()
        self.odom_pose.pre_timestamp = self.get_clock().now()
        
        
        ######### parameter initial ######### 
        self.odo_l = 0.
        self.odo_r = 0.
        self.trans_vel = 0.
        self.orient_vel = 0.
        ####################################
        
        self.timer_param_set = self.create_timer(1, self.update_params)
        self.timer_odom_set = self.create_timer(1/2, self.update_odometry)
        
    def update_params(self) :
        self.odo_l -= 0.
        self.odo_r -= 0.
        self.trans_vel += 0.01
        self.orient_vel += 0.01  
        
        
    def update_odometry(self):
        self.odom_pose.timestamp = self.get_clock().now()
        dt = (self.odom_pose.timestamp - self.odom_pose.pre_timestamp).nanoseconds * 1e-9
        self.odom_pose.pre_timestamp = self.odom_pose.timestamp

        d_x = self.trans_vel * math.cos(self.odom_pose.theta) 
        d_y = self.trans_vel * math.sin(self.odom_pose.theta) 
        self.odom_pose.x += d_x * dt
        self.odom_pose.y += d_y * dt
        #print('ODO L:%.2f, R:%.2f, V:%.2f, W=%.2f --> X:%.2f, Y:%.2f, Theta:%.2f'
        #  %(odo_l, odo_r, trans_vel, orient_vel, 
        #  self.odom_pose.x,self.odom_pose.y,self.odom_pose.theta))
        q = self.quaternion_from_euler(0, 0, self.odom_pose.theta)

        # self.odom_vel.x = trans_vel
        # self.odom_vel.y = 0.
        # self.odom_vel.w = orient_vel

        timestamp_now = self.get_clock().now().to_msg()
        # Set odometry data
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.header.stamp = timestamp_now
        odom.pose.pose.position.x = self.odom_pose.x
        odom.pose.pose.position.y = self.odom_pose.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = self.trans_vel
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.orient_vel

        ### publish ###
        self.pub_Odom.publish(odom)

        # Set odomTF data
        odom_tf = TransformStamped()
        odom_tf.header.frame_id = odom.header.frame_id
        odom_tf.child_frame_id = odom.child_frame_id
        odom_tf.header.stamp = timestamp_now

        odom_tf.transform.translation.x = odom.pose.pose.position.x
        odom_tf.transform.translation.y = odom.pose.pose.position.y
        odom_tf.transform.translation.z = odom.pose.pose.position.z
        odom_tf.transform.rotation = odom.pose.pose.orientation
        
        ### publish ###
        self.pub_OdomTF.sendTransform(odom_tf)
        
        self.get_logger().info(f'odom : {odom}   odom_tf : {odom_tf}')
        
    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        quat = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * sr - sy * sp * cr
        q[1] = sy * cp * sr + cy * sp * cr
        q[2] = sy * cp * cr - cy * sp * sr
        q[3] = cy * cp * cr + sy * sp * sr

        return q
        
    
class OdomPose(object):
  x = 0.0
  y = 0.0
  theta = 0.0
  timestamp = 0
  pre_timestamp = 0
        

def main(args=None):
    rclpy.init(args=args)
    node = OdomTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()