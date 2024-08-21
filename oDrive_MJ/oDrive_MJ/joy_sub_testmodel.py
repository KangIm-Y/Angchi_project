#joy_sub_testmodel
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import odrive
from odrive.enums import CONTROL_MODE_VELOCITY_CONTROL
from odrive.enums import INPUT_MODE_VEL_RAMP
import time
from sensor_msgs.msg import Joy


class Subscriber(Node):

    def __init__(self):
        super().__init__('joy')

        self.my_drive = odrive.find_any()
        self.calibration()

        qos_profile = QoSProfile(depth=10)
        self.motor_control_sub = self.create_subscription(
            Joy, 
            'joy',
            self.subscribe_joy_message,  
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

    def subscribe_joy_message(self, msg): 
        
        axes = msg.axes
        buttons = msg.buttons

        self.get_logger().info(f'Axes: {axes}')
        self.get_logger().info(f'Buttons: {buttons}')

        self.my_drive.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        self.my_drive.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

        self.my_drive.axis0.controller.config.vel_ramp_rate = 5
        self.my_drive.axis1.controller.config.vel_ramp_rate = 5

        self.my_drive.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP
        self.my_drive.axis1.controller.config.input_mode = INPUT_MODE_VEL_RAMP

        # Default to zero velocity
        self.my_drive.axis0.controller.input_vel = 0
        self.my_drive.axis1.controller.input_vel = 0
        self.get_logger().info('GABOJA GO')

        if buttons[6] == 1 & buttons[7] == 1:  # Button index 0
            self.my_drive.axis0.controller.input_vel = 0
            self.my_drive.axis1.controller.input_vel = 0
            self.get_logger().info('STOP')

        else:

            self.my_drive.axis0.controller.input_vel = axes[1]*4
            self.my_drive.axis1.controller.input_vel = -axes[4]*4
            self.get_logger().info('CONTROL GOGO')


        time.sleep(0.1)


def main():
    rclpy.init()
    sub = Subscriber()
    rclpy.spin(sub)
    sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
