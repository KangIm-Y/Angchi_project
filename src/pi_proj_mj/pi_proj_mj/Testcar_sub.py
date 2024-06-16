#Testcar_sub.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Int32MultiArray
from odrive.enums import InputMode
from odrive.enums import CONTROL_MODE_VELOCITY_CONTROL
from odrive.enums import INPUT_MODE_VEL_RAMP

import odrive
import time


class Testcar_sub(Node):

    def __init__(self):
        super().__init__('TEST_CAR_SUB')

        self.my_drive = odrive.find_any()
        self.calibration()

        qos_profile = QoSProfile(depth=10)
        self.motor_control_sub = self.create_subscription(
            Int32MultiArray,
            'ANG',
            self.subscribe_topic_message,
            qos_profile)
        
    def calibration(self):
        self.get_logger().info('Calibration START')
        self.my_drive.axis0.requested_state = odrive.enums.AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        self.my_drive.axis1.requested_state = odrive.enums.AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        while self.my_drive.axis0.current_state != odrive.enums.AXIS_STATE_IDLE:
            time.sleep(0.1)
        while self.my_drive.axis1.current_state != odrive.enums.AXIS_STATE_IDLE:
            time.sleep(0.1)
            
        self.my_drive.axis0.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL
        self.my_drive.axis1.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL

        self.get_logger().info('Calibration COMPLETE.')




    def subscribe_topic_message(self,msg):

        self.my_drive.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        self.my_drive.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL


        self.my_drive.axis0.controller.config.vel_ramp_rate = 10
        self.my_drive.axis1.controller.config.vel_ramp_rate = 10


        self.my_drive.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP
        self.my_drive.axis1.controller.config.input_mode = INPUT_MODE_VEL_RAMP

        #self.my_drive.axis0.controller.input_vel = 1
        #self.my_drive.axis1.controller.input_vel = 1

        self.get_logger().info('GOGO')


        #CONTROL
        self.my_drive.axis0.controller.input_vel = -msg.data[0]
        self.my_drive.axis1.controller.input_vel = msg.data[1]
        time.sleep(0.1)


            
def main():
    rclpy.init()
    sub = Testcar_sub()
    rclpy.spin(sub)
    sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
