import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32MultiArray
from odrive.enums import InputMode, CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_VEL_RAMP

import odrive
import time

class Odrive_car(Node):
    def __init__(self):
        super().__init__('test_car_sub')
        self.my_drive = odrive.find_any()
        self.calibration()
        qos_profile = QoSProfile(depth=10)
        

        #명령받는 Subscriber
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'Odrive_control',
            self.subscribe_topic_message,
            qos_profile)
        

        #Encoder Publisher
        self.publisher = self.create_publisher(
            Float32MultiArray,
            'Odrive_encoder', 
            qos_profile
        )

        self.timer = self.create_timer(0.1, self.encoder_callback)
        self.timer2 = self.create_timer(0.1, self.read_error)  

        self.mode = 'pos'
        self.cur_mode = 'pos'
        self.pos_axis0 = 0.0
        self.pos_axis1 = 0.0

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
        self.get_logger().info('Received data : {}'.format(msg.data))
        movedata = msg.data[1:]   
        movedata[0], movedata[1] = movedata[1], movedata[0]
        movedata[0] = -movedata[0] # axis1 방향 전환 -,+
        
        if msg.data[0] == 1:    #Ramped 속도 제어 모드
            self.mode = 'vel'
            
        elif msg.data[0] == 2:  #상대 위치제어 모드 : Trajectory
            self.mode = 'pos'

        else:
            self.get_logger().warn('Invalid control mode received: {}'.format(msg.data[0]))

        self.motor_mode_set()
        self.motor_run(movedata)


    def motor_mode_set(self):
        if self.mode == "vel" and self.cur_mode != "vel":

            self.my_drive.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            self.my_drive.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            self.my_drive.axis0.controller.config.vel_ramp_rate = 10
            self.my_drive.axis1.controller.config.vel_ramp_rate = 10
            self.my_drive.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP
            self.my_drive.axis1.controller.config.input_mode = INPUT_MODE_VEL_RAMP
            self.get_logger().info('Change mode to velocity')
            self.cur_mode = "vel"
            
        elif self.mode == "pos" and self.cur_mode != "pos":

            #PASSTHROUGH Mode

            self.my_drive.axis0.controller.config.input_mode = 1
            self.my_drive.axis1.controller.config.input_mode = 1
         
            ############################ ENCODER OFFSET ##################################
            self.pos_axis0_offset = self.my_drive.axis0.encoder.pos_estimate
            self.pos_axis1_offset = self.my_drive.axis1.encoder.pos_estimate
            
            # INPUT POS에 현재 엔코더 값 넣기
            self.my_drive.axis0.controller.input_pos = self.pos_axis0_offset 
            self.my_drive.axis1.controller.input_pos = self.pos_axis1_offset
            self.get_logger().info(f'Change Offset : {self.pos_axis0_offset} , {self.pos_axis1_offset}')  

            
            self.my_drive.axis0.controller.config.control_mode = 3
            self.my_drive.axis1.controller.config.control_mode = 3

            self.my_drive.axis0.controller.config.input_mode = InputMode.TRAP_TRAJ
            self.my_drive.axis1.controller.config.input_mode = InputMode.TRAP_TRAJ
            self.get_logger().info('Change mode to position')
            self.cur_mode = "pos"


    def motor_run(self, data):

        if self.cur_mode == "pos":          #Trajectory Control 
            self.my_drive.axis0.controller.input_pos = data[0] + self.pos_axis0_offset
            self.my_drive.axis1.controller.input_pos = data[1] + self.pos_axis1_offset
            # self.get_logger().info(f'Position control set : axis0 = {data[0]}, axis1 = {data[1]}')
        
        elif self.cur_mode == "vel":        # Ramped Velocity Control 
            self.my_drive.axis0.controller.input_vel = data[0]
            self.my_drive.axis1.controller.input_vel = data[1]
            # self.get_logger().info(f'Velocity control set : axis0 = {data[0]}, axis1 = {data[1]}')
        
        else : 
            self.get_logger().fatal(f'Invalid mode!! reset to vel mode')
            self.mode = "vel"
        self.encoder_check()
        

    def encoder_check(self):
        self.pos_axis0 = self.my_drive.axis0.encoder.pos_estimate
        self.pos_axis1 = self.my_drive.axis1.encoder.pos_estimate
        #self.get_logger().info('Encoder positions: axis0 = {}, axis1 = {}'.format(self.pos_axis0, self.pos_axis1))


#모터 엔코더 값 받아오는 콜백함수
    def encoder_callback(self):
        msg = Float32MultiArray()
        msg.data = [self.pos_axis0, self.pos_axis1]
        self.publisher.publish(msg)
        # self.get_logger().info(f'ENCODER Value: {self.pos_axis0} , {self.pos_axis1}')  

#ERROR 검출 함수
    def read_error(self):
        axis_error0 = self.my_drive.axis0.error
        axis_error1 = self.my_drive.axis1.error
        motor_error0 = self.my_drive.axis0.motor.error
        motor_error1 = self.my_drive.axis1.motor.error
        controller_error0 = self.my_drive.axis0.controller.error
        controller_error1 = self.my_drive.axis1.controller.error
        encoder_error0 = self.my_drive.axis0.encoder.error
        encoder_error1 = self.my_drive.axis1.encoder.error
        Errors = {'axis_error0' : axis_error0,
                  'axis_error1' : axis_error1,
                  'motor_error0' : motor_error0,
                  'motor_error1' : motor_error1,
                  'controller_error0' : controller_error0,
                  'controller_error1' : controller_error1,
                  'encoder_error0' : encoder_error0,
                  'encoder_error1' : encoder_error1}
        for i in Errors.keys():
            if Errors[i] != 0:
                self.get_logger().fatal(f"{i} : {Errors[i]}")



def main(args=None):
    rclpy.init(args=args)
    sub = Odrive_car()
    rclpy.spin(sub)
    sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
