import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from odrive.enums import InputMode, CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_VEL_RAMP
import odrive
import time

class SharedEncoderData: #Encoder Data 공유를 위한 Initializing
    def __init__(self):
        self.pos_axis0 = 0.0
        self.pos_axis1 = 0.0

shared_data = SharedEncoderData()

class Testcar_sub(Node):
    def __init__(self):
        super().__init__('test_car_sub')
        self.my_drive = odrive.find_any() # ODrive 찾기
        self.calibration() # 모터 calibration 

        qos_profile = QoSProfile(depth=10)
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'ANG',
            self.subscribe_topic_message,
            qos_profile
        )

    def calibration(self):
        self.get_logger().info('----------------Calibration START----------------------------')
        self.my_drive.axis0.requested_state = odrive.enums.AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        self.my_drive.axis1.requested_state = odrive.enums.AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        while self.my_drive.axis0.current_state != odrive.enums.AXIS_STATE_IDLE:
            time.sleep(0.1)
        while self.my_drive.axis1.current_state != odrive.enums.AXIS_STATE_IDLE:
            time.sleep(0.1)
        self.my_drive.axis0.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL
        self.my_drive.axis1.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL
        self.get_logger().info('----------------Calibration COMPLETE-------------------------')

    def subscribe_topic_message(self, msg):
        if msg.data[0] == 1:
            self.get_logger().info('----------------Ramped Velocity Mode------------------------')
            self.my_drive.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            self.my_drive.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            self.my_drive.axis0.controller.config.vel_ramp_rate = 10
            self.my_drive.axis1.controller.config.vel_ramp_rate = 10
            self.my_drive.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP
            self.my_drive.axis1.controller.config.input_mode = INPUT_MODE_VEL_RAMP

            shared_data.pos_axis0 = self.my_drive.axis0.encoder.pos_estimate
            shared_data.pos_axis1 = self.my_drive.axis1.encoder.pos_estimate
            self.get_logger().info('Encoder positions: axis0 = {}, axis1 = {}'.format(shared_data.pos_axis0, shared_data.pos_axis1))

            msg.data[2] = -msg.data[2]
            self.my_drive.axis0.controller.input_vel = - msg.data[2]
            self.my_drive.axis1.controller.input_vel = - msg.data[1]
            self.get_logger().info('Velocity control set: axis0 = {}, axis1 = {}'.format(msg.data[1], msg.data[2]))

        elif msg.data[0] == 2:
            self.get_logger().info('----------------Trajectory Mode------------------------')
            self.my_drive.axis0.controller.config.input_mode = 1
            self.my_drive.axis1.controller.config.input_mode = 1

            shared_data.pos_axis0 = self.my_drive.axis0.encoder.pos_estimate
            shared_data.pos_axis1 = self.my_drive.axis1.encoder.pos_estimate
            self.get_logger().info('Encoder positions: axis0 = {}, axis1 = {}'.format(shared_data.pos_axis0, shared_data.pos_axis1))

            self.my_drive.axis0.controller.input_pos = shared_data.pos_axis0 
            self.my_drive.axis1.controller.input_pos = shared_data.pos_axis1

            self.my_drive.axis0.controller.config.control_mode = 3
            self.my_drive.axis1.controller.config.control_mode = 3

            self.my_drive.axis0.controller.config.input_mode = InputMode.TRAP_TRAJ
            self.my_drive.axis1.controller.config.input_mode = InputMode.TRAP_TRAJ

            shared_data.pos_axis0 = self.my_drive.axis0.encoder.pos_estimate
            shared_data.pos_axis1 = self.my_drive.axis1.encoder.pos_estimate
            self.get_logger().info('Encoder positions: axis0 = {}, axis1 = {}'.format(shared_data.pos_axis0, shared_data.pos_axis1))

            msg.data[2] = -msg.data[2]
            self.my_drive.axis0.controller.move_incremental(msg.data[1], False)
            self.my_drive.axis1.controller.move_incremental(msg.data[2], False)
            self.get_logger().info('Position control set: axis0 = {}, axis1 = {}'.format(msg.data[1], msg.data[2]))

        else:
            self.get_logger().info('Invalid control mode received: {}'.format(msg.data[0]))

class encoder_feedback(Node):
    def __init__(self):
        super().__init__('encoder_feedback')
        qos_profile = QoSProfile(depth=10)
        self.publisher = self.create_publisher(
            Float32MultiArray,
            'encoder', 
            qos_profile
        )
        self.timer = self.create_timer(0.1, self.encoder_callback) 

    def encoder_callback(self):
        msg = Float32MultiArray()
        msg.data = [shared_data.pos_axis0, shared_data.pos_axis1]
        self.publisher.publish(msg)
        self.get_logger().info('ENCODER Value: {0}'.format(msg.data))  

def main(args=None):
    rclpy.init(args=args)
    sub = Testcar_sub() # 차체 Subscriber
    encoder = encoder_feedback() # Encoder Feedback Publisher

    try:
        while rclpy.ok():
            rclpy.spin_once(sub)
            rclpy.spin_once(encoder)
            
    except KeyboardInterrupt:
        sub.get_logger().info('Keyboard Interrupt (SIGINT)')
        encoder.get_logger().info('Keyboard Interrupt (SIGINT)')

    finally:
        sub.destroy_node()
        encoder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
