#Testcar_sub_mode.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32MultiArray
from odrive.enums import InputMode, CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_VEL_RAMP

import odrive
import time

class Testcar_sub(Node):
    def __init__(self):
        super().__init__('test_car_sub')
        self.my_drive = odrive.find_any()
        self.calibration()
        qos_profile = QoSProfile(depth=10)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'Odrive_control',
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

    def subscribe_topic_message(self, msg):
        
        #Ramped 속도 제어 모드
        if msg.data[0] == 1:

            self.my_drive.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            self.my_drive.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            self.my_drive.axis0.controller.config.vel_ramp_rate = 10
            self.my_drive.axis1.controller.config.vel_ramp_rate = 10
            self.my_drive.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP
            self.my_drive.axis1.controller.config.input_mode = INPUT_MODE_VEL_RAMP
            
            ############################ ENCODER Feedback ##################################
            pos_axis0 = self.my_drive.axis0.encoder.pos_estimate
            pos_axis1 = self.my_drive.axis1.encoder.pos_estimate
            self.get_logger().info('Encoder positions: axis0 = {}, axis1 = {}'.format(pos_axis0, pos_axis1))

            #msg : 초 당 회전수
            ########################## Ramped Velocity Control #############################
            msg.data[1] = -msg.data[1]
            
            self.my_drive.axis0.controller.input_vel = msg.data[1]
            self.my_drive.axis1.controller.input_vel = msg.data[2]
            self.get_logger().info('Velocity control set: axis0 = {}, axis1 = {}'.format(msg.data[1], msg.data[2]))
            self.get_logger().info('Moving...by Ramped Velocity')
            
        elif msg.data[0] == 2: #상대 위치제어 모드 : Trajectory

            #PASSTHROUGH Mode

            self.my_drive.axis0.controller.config.input_mode = 1
            self.my_drive.axis1.controller.config.input_mode = 1
         
            ############################ ENCODER Feedback ##################################
            pos_axis0 = self.my_drive.axis0.encoder.pos_estimate
            pos_axis1 = self.my_drive.axis1.encoder.pos_estimate
            self.get_logger().info('Encoder positions: axis0 = {}, axis1 = {}'.format(pos_axis0, pos_axis1))
            
            # INPUT POS에 현재 엔코더 값 넣기
            self.my_drive.axis0.controller.input_pos = pos_axis0 
            self.my_drive.axis1.controller.input_pos = pos_axis1
            
            self.my_drive.axis0.controller.config.control_mode = 3
            self.my_drive.axis1.controller.config.control_mode = 3

            self.my_drive.axis0.controller.config.input_mode = InputMode.TRAP_TRAJ
            self.my_drive.axis1.controller.config.input_mode = InputMode.TRAP_TRAJ
            ############################ ENCODER Feedback ##################################
            pos_axis0 = self.my_drive.axis0.encoder.pos_estimate
            pos_axis1 = self.my_drive.axis1.encoder.pos_estimate
            self.get_logger().info('Encoder positions: axis0 = {}, axis1 = {}'.format(pos_axis0, pos_axis1))
            
            ##########################Trajecotory Mode Control #############################
            msg.data[1] = -msg.data[1]

            #msg,data에 얼마나 움직이고 싶은지 작성(엔코더 분해능 값이 들어감). 상대위치 제어
            self.my_drive.axis0.controller.move_incremental(msg.data[1], True)
            self.my_drive.axis1.controller.move_incremental(msg.data[2], True)
            self.get_logger().info('FINISH POSITION MODE')
            self.get_logger().info('Position control set: axis0 = {}, axis1 = {}'.format(msg.data[1], msg.data[2]))

        else:
            self.get_logger().info('Invalid control mode received: {}'.format(msg.data[0]))

def main(args=None):
    rclpy.init(args=args)
    sub = Testcar_sub()
    rclpy.spin(sub)
    sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()