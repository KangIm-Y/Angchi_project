import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Int32MultiArray
from example_interfaces.srv import SetBool  # 서비스 타입은 필요에 맞게 조정하세요
from odrive.enums import CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_VEL_RAMP

import odrive
import time

class Velocity_sub(Node):
    def __init__(self):
        super().__init__('velocity_sub')
        self.my_drive = odrive.find_any()
        self.calibration()
        
        # QoSProfile 설정
        qos_profile = QoSProfile(depth=10)
        
        # Subscription 설정
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'ANG',
            self.subscribe_topic_message,
            qos_profile)
        
        # 서비스 서버 설정
        self.srv = self.create_service(SetBool, 'get_encoder_values', self.get_encoder_values_callback)

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
        if msg.data[0] == 1:
            self.my_drive.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            self.my_drive.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            self.my_drive.axis0.controller.config.vel_ramp_rate = 10
            self.my_drive.axis1.controller.config.vel_ramp_rate = 10
            self.my_drive.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP
            self.my_drive.axis1.controller.config.input_mode = INPUT_MODE_VEL_RAMP
            
            # 엔코더 피드백
            pos_axis0 = self.my_drive.axis0.encoder.pos_estimate
            pos_axis1 = self.my_drive.axis1.encoder.pos_estimate
            self.get_logger().info('Encoder positions: axis0 = {}, axis1 = {}'.format(pos_axis0, pos_axis1))

            msg.data[2] = -msg.data[2]
            
            # 초당 회전수
            self.my_drive.axis0.controller.input_vel = msg.data[1]
            self.my_drive.axis1.controller.input_vel = msg.data[2]
            self.get_logger().info('Velocity control set: axis0 = {}, axis1 = {}'.format(msg.data[1], msg.data[2]))
            
        else:
            self.get_logger().info('Invalid control mode received: {}'.format(msg.data[0]))

    def get_encoder_values_callback(self, request, response):
        # 엔코더 값 읽기
        pos_axis0 = self.my_drive.axis0.encoder.pos_estimate
        pos_axis1 = self.my_drive.axis1.encoder.pos_estimate
        
        # 응답 설정
        response.success = True
        response.message = f'Encoder positions: axis0 = {pos_axis0}, axis1 = {pos_axis1}'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = Velocity_sub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
