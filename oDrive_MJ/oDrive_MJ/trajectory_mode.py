import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Int32MultiArray
from odrive.enums import InputMode

import odrive
import time

class Trajectory_sub(Node):
    def __init__(self):
        super().__init__('trajectory_sub')
        self.my_drive = odrive.find_any()
        self.calibration()

        qos_profile = QoSProfile(depth=10)
        
        # Subscription 설정
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'ANG',
            self.subscribe_topic_message,
            qos_profile)
        
        # 서비스 서버 설정
        self.srv = self.create_service(Empty, 'get_encoder_values', self.get_encoder_values_callback)

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
        if msg.data[0] == 2:  # 상대 위치제어 모드
            self.my_drive.axis0.controller.config.input_mode = InputMode.TRAP_TRAJ
            self.my_drive.axis1.controller.config.input_mode = InputMode.TRAP_TRAJ
            ######################### ENCODER Feedback #############################
            pos_axis0 = self.my_drive.axis0.encoder.pos_estimate
            pos_axis1 = self.my_drive.axis1.encoder.pos_estimate
            self.get_logger().info('Encoder positions: axis0 = {}, axis1 = {}'.format(pos_axis0, pos_axis1))

            msg.data[2] = -msg.data[2]
            # msg 데이터에 얼마나 움직이고 싶은지 작성 (엔코더 분해능 값이 들어감)
            self.my_drive.axis0.controller.move_incremental(msg.data[1], True)
            self.my_drive.axis1.controller.move_incremental(msg.data[2], True)
            self.get_logger().info('Position control set: axis0 = {}, axis1 = {}'.format(msg.data[1], msg.data[2]))
        else:
            self.get_logger().info('Invalid control mode received: {}'.format(msg.data[0]))

    def get_encoder_values_callback(self, request, response):
        # 엔코더 값 읽기
        pos_axis0 = self.my_drive.axis0.encoder.pos_estimate
        pos_axis1 = self.my_drive.axis1.encoder.pos_estimate

        self.get_logger().info(f'Encoder positions: axis0 = {pos_axis0}, axis1 = {pos_axis1}')
        
        # 서비스 응답 
        response.success = True
        response.message = f'Encoder positions: axis0 = {pos_axis0}, axis1 = {pos_axis1}'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = Trajectory_sub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
