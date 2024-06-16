#mode_control_meja.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from odrive.enums import InputMode
from odrive.enums import CONTROL_MODE_VELOCITY_CONTROL
from odrive.enums import CONTROL_MODE_POSITION_CONTROL
from odrive.enums import INPUT_MODE_VEL_RAMP
from odrive.enums import INPUT_MODE_TRAP_TRAJ

import odrive
import time

class ModeA(Node):

    def __init__(self):
        super().__init__('modeA')

        self.my_drive = odrive.find_any()

        qos_profile = QoSProfile(depth=10)
        self.subscription = self.create_subscription(
            String,
            'modeA',
            self.modeA_control,
            qos_profile)

    def modeA_control(self, msg):
        self.get_logger().info('Received message: {0}'.format(msg.data))

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

        self.my_drive.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        self.my_drive.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

        self.my_drive.axis0.controller.config.vel_ramp_rate = 5
        self.my_drive.axis1.controller.config.vel_ramp_rate = 5

        self.my_drive.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP
        self.my_drive.axis1.controller.config.input_mode = INPUT_MODE_VEL_RAMP

        mode_data = msg.data
        if mode_data == 'front':
            self.my_drive.axis0.controller.input_vel = 10
            self.my_drive.axis1.controller.input_vel = 10
            time.sleep(0.1)
            self.get_logger().info('FRONT')
        elif mode_data == 'back':
            self.my_drive.axis0.controller.input_vel = -10
            self.my_drive.axis1.controller.input_vel = -10
            time.sleep(0.1)
            self.get_logger().info('BACK')
        elif mode_data == 'left':
            self.my_drive.axis0.controller.input_vel = 10
            self.my_drive.axis1.controller.input_vel = -10
            time.sleep(0.1)
            self.get_logger().info('LEFT')
        elif mode_data == 'right':
            self.my_drive.axis0.controller.input_vel = -10
            self.my_drive.axis1.controller.input_vel = 10
            time.sleep(0.1)
            self.get_logger().info('RIGHT')
        elif mode_data == 'SL':
            self.my_drive.axis0.controller.input_vel = 10
            self.my_drive.axis1.controller.input_vel = 0
            time.sleep(0.1)
            self.get_logger().info('STOP AND LEFT')
        elif mode_data == 'SR':
            self.my_drive.axis0.controller.input_vel = 0
            self.my_drive.axis1.controller.input_vel = -10
            time.sleep(0.1)
            self.get_logger().info('STOP AND RIGHT')
        elif mode_data == 'stop':
            self.my_drive.axis0.controller.input_vel = 0
            self.my_drive.axis1.controller.input_vel = 0
            time.sleep(0.1)
            self.get_logger().info('STOP')

class ModeB(Node):

    def __init__(self):
        super().__init__('modeB')

        self.my_drive = odrive.find_any()

        qos_profile = QoSProfile(depth=10)
        self.subscription = self.create_subscription(
            String,
            'modeB',
            self.modeB_control,
            qos_profile)

    def modeB_control(self, msg):
        self.get_logger().info('Received message: {0}'.format(msg.data))
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
        self.my_drive.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        self.my_drive.axis1.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        self.my_drive.axis0.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
        self.my_drive.axis1.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ

        self.my_drive.odrv0.axis0.trap_traj.config.vel_limit = 30.0
        self.my_drive.odrv1.axis0.trap_traj.config.vel_limit = 30.0

        self.my_drive.axis0.trap_traj.config.accel_limit = 2.0   
        self.my_drive.axis1.trap_traj.config.accel_limit = 2.0  
        self.my_drive.axis0.trap_traj.config.decel_limit = 2.0
        self.my_drive.axis1.trap_traj.config.decel_limit = 2.0
        self.my_drive.axis0.controller.config.inertia = 0
        self.my_drive.axis1.controller.config.inertia = 0       

        self.my_drive.axis0.motor.config.current_lim = 20.0
        self.my_drive.axis1.motor.config.current_lim = 20.0
        self.my_drive.axis0.controller.config.vel_limit = 50.0
        self.my_drive.axis1.controller.config.vel_limit = 50.0

        mode_data = msg.data
        if mode_data == 'front':
            self.my_drive.axis0.controller.move_incremental(20, True)
            time.sleep(0.1)
            self.my_drive.axis1.controller.move_incremental(20, True)
            time.sleep(0.1)
            self.get_logger().info('FRONT')
        elif mode_data == 'back':
            self.my_drive.axis0.controller.move_incremental(-20, True)
            time.sleep(0.1)
            self.my_drive.axis1.controller.move_incremental(-20, True)      
            time.sleep(0.1)
            self.get_logger().info('BACK')
        elif mode_data == 'left':
            self.my_drive.axis0.controller.move_incremental(20, True)
            time.sleep(0.1)
            self.my_drive.axis1.controller.move_incremental(-20, True)
            time.sleep(0.1)
            self.get_logger().info('LEFT')
        elif mode_data == 'right':
            self.my_drive.axis0.controller.move_incremental(-20, True)
            time.sleep(0.1)
            self.my_drive.axis1.controller.move_incremental(20, True)
            time.sleep(0.1)
            self.get_logger().info('RIGHT')
        elif mode_data == 'SL':
            self.my_drive.axis0.controller.move_incremental(20, True)
            time.sleep(0.1)
            self.get_logger().info('STOP AND LEFT')
        elif mode_data == 'SR':
            self.my_drive.axis1.controller.move_incremental(20, True)          
            time.sleep(0.1)
            self.get_logger().info('STOP AND RIGHT')
        elif mode_data == 'stop':
            self.my_drive.axis0.controller.input_vel = 0
            self.my_drive.axis1.controller.input_vel = 0
            time.sleep(0.1)
            self.get_logger().info('STOP')

def main():
    rclpy.init()
    
    modeA_node = ModeA()
    modeB_node = ModeB()
    
    rclpy.spin(modeA_node)
    rclpy.spin(modeB_node)
    
    modeA_node.destroy_node()
    modeB_node.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
