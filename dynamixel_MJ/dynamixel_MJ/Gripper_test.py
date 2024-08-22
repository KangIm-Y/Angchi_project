#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from .robotis_def import *
from .protocol2_packet_handler import * 
from .packet_handler import * 
from .port_handler import * 
from std_srvs.srv import SetBool
#from dynamixel_sdk import * 

# Control table address
ADDR_OPERATING_MODE = 11              
ADDR_TORQUE_ENABLE = 64               
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_CURRENT = 126

# Protocol version
PROTOCOL_VERSION = 2.0            
BAUDRATE = 115200

# Default setting
MAX_POSITION_VALUE = 1048575             
EXT_POSITION_CONTROL_MODE = 4                 

# Dynamixel IDs
DXL1_ID = 4
DXL2_ID = 5
DXL3_ID = 6  

DEVICENAME1 = '/dev/ttyUSB0'   
TORQUE_ENABLE = 1                 
TORQUE_DISABLE = 0                 
DXL_MOVING_STATUS_THRESHOLD = 20                

# Initialize PortHandler instance
portHandler = PortHandler(DEVICENAME1)

# Initialize PacketHandler instance
packetHandler = PacketHandler(PROTOCOL_VERSION)

#4~5 joint
OFFSET_4 = 347.488
OFFSET_5 = 114

class TripSub(Node):
    def __init__(self):
        super().__init__('dynamixel_subscriber')
        
        self.create_subscription(Int32MultiArray, 'dynamixel_double', self.callback, 10)
        self.current = 0

        # OPEN PORT
        if portHandler.openPort():
            self.get_logger().info('Succeeded to open the port')
        else:
            self.get_logger().error('Failed to open the port')
            raise Exception('Failed to open the port')

        # BAUD RATE
        if portHandler.setBaudRate(BAUDRATE):
            self.get_logger().info('Succeeded to change the baudrate')
        else:
            self.get_logger().error('Failed to change the baudrate')
            raise Exception('Failed to change the baudrate')

        # Enable Torque
        for dxl_id in [DXL1_ID, DXL2_ID, DXL3_ID]:
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE, EXT_POSITION_CONTROL_MODE)
            self.get_logger().info('COUNT1')
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(f'ID={dxl_id} Failed to change operating mode: {packetHandler.getTxRxResult(dxl_comm_result)}')
                raise Exception(f'ID={dxl_id} Failed to change operating mode')
            else:
                self.get_logger().info(f'ID={dxl_id} Operating mode changed to extended position control mode.')

            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            self.get_logger().info('COUNT2')
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(f'ID={dxl_id} Failed to enable torque: {packetHandler.getTxRxResult(dxl_comm_result)}')
                raise Exception(f'ID={dxl_id} Failed to enable torque')
            else:
                self.get_logger().info(f'ID={dxl_id} Dynamixel has been successfully connected')

    def callback(self, msg):
        # Write Goal Position
        goal_position1 = int((msg.data[0] + OFFSET_4) / 0.088)
        goal_position2 = int((msg.data[1] + OFFSET_5) / 0.088)

        # Set goal positions
        for dxl_id, goal_position in zip([DXL1_ID, DXL2_ID], [goal_position1, goal_position2]):
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, goal_position)
            self.get_logger().info('COUNT3')
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(f'Failed to set goal position ID{dxl_id}: {packetHandler.getTxRxResult(dxl_comm_result)}')

        # Update current and call grip_callback
        self.torque()  # Update self.current
        self.grip_callback(msg)

    def grip_callback(self, msg):
        angle_step = 40
        self.initial_position = int(250 / 0.088)
        final_position = int(-10 / 0.088)
        

        if msg.data[2] == 1:
            self.gripper(self.initial_position)
          
            for goal_position in range(int(self.initial_position), int(final_position), -angle_step):
                self.gripper(goal_position)
                #time.sleep(0.01)
                self.torque()  # Update self.current
                time.sleep(0.01)
                
                if self.current <= -110: #성공
                    self.get_logger().info('success')
                    self.get_logger().info(f'Present current: {self.current}')
                
                    break
                
                elif self.girpper(self.dxl_present_current) < goal_position:# 타켓 도달 안 한 경우
                    self.get_logger().info('success')

                    #겟로거로 계속 진행시켜
                    
                    pass

                else: #타겟 도달 하고 물체도 못찾음

                    self.get_logger().info('FAIL')
                    self.get_logger().info(f'Present current: {self.current}')
                    self.gripper(self.initial_position)
                    break



                #if self.current >=  4200000000 and self.current <= 6418370:
                # if self.current <=-0.3000000115968000:

                #     self.get_logger().info('success')
                #     #self.get_logger().info(f'Present current: {self.current}')
                
                # # elif self.current >= 0:  # 역전류 흐르는 경우
                # #      self.gripper(initial_position)
                # #      #self.get_logger().info(f'Present current: {self.current}')
                # #      self.get_logger().info('FAIL_REVERSE')
                     
                # else:
                #     self.gripper(initial_position)
                #     #self.get_logger().info(f'Present current: {self.current}')

                #     self.get_logger().info('FAIL')
        
        else:
            self.gripper(self.initial_position)
            self.get_logger().info('Gripper released')

    def gripper(self, goal_position):
        # ID=3 WRITE
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_GOAL_POSITION, goal_position)
        self.get_logger().info('COUNT4')
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Failed to set goal position ID3: {packetHandler.getTxRxResult(dxl_comm_result)}')
        else:
            self.get_logger().info(f'ID3 goal position: {round(goal_position)}')

        # GRIPPER READ
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL3_ID, ADDR_PRESENT_POSITION)
        if dxl_comm_result == COMM_SUCCESS:
            self.get_logger().info(f'ID3 ENCODER: {dxl_present_position * 0.088}')
        else:
            self.get_logger().error(f'Failed to read present position ID3: {packetHandler.getTxRxResult(dxl_comm_result)}')

    def torque(self):
        # Read present current from the Dynamixel motor
        CURRENT_MAX = 65536
        dxl_present_current, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL3_ID, ADDR_PRESENT_CURRENT)
        
        if dxl_comm_result == COMM_SUCCESS:
            #if dxl_present_current >= int(CURRENT_MAX / 2):
                #self.dxl_present_current = abs(CURRENT_MAX - dxl_present_current)
                self.current_init = (dxl_present_current & 0XFFFF)
                
                
            #else:
                #self.current = dxl_present_current
                #self.get_logger().info(f'Present current: {self.dxl_present_current}')

            
            ####
                if self.current_init >= 65536/2:
                   self.current = (65536 - self.current_init)*(-1)
                   print(f'Present current: {self.current}')

  
                else:
                    self.current = self.current_init
                    print(f'Present current: {self.current}')
           

        else:
            self.get_logger().error(f'Failed to read present current (Error: {dxl_error})')





def main(args=None):
    rclpy.init(args=args)
    dynamixel_subscriber = TripSub() # 4~6 joint

    try:

        while(1):
            rclpy.spin_once(dynamixel_subscriber)
    except KeyboardInterrupt:
        dynamixel_subscriber.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        dynamixel_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
