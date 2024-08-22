#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from .robotis_def import *
from .protocol2_packet_handler import * 
from .packet_handler import * 
from .port_handler import * 
from custom_interfaces.srv import PositionService
from std_srvs.srv import SetBool
import time

# Control table address
ADDR_OPERATING_MODE         = 11              
ADDR_TORQUE_ENABLE          = 64               
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_POSITION       = 132
ADDR_PRESENT_CURRENT = 126
# Protocol version
PROTOCOL_VERSION            = 2.0            
BAUDRATE                    = 57600   

# Default setting
MAX_POSITION_VALUE          = 1048575             
EXT_POSITION_CONTROL_MODE   = 4                 

ESC_ASCII_VALUE             = 0x1b
SPACE_ASCII_VALUE           = 0x20

# Dynamixel IDs
DXL1_ID                     = 4
DXL2_ID                     = 5
DXL3_ID                     = 6  

DEVICENAME1                 = '/dev/ttyUSB0'   
TORQUE_ENABLE               = 1                 
TORQUE_DISABLE              = 0                 
DXL_MOVING_STATUS_THRESHOLD = 20                

# Initialize PortHandler instance
portHandler = PortHandler(DEVICENAME1)

# Initialize PacketHandler instance
packetHandler = PacketHandler(PROTOCOL_VERSION)

class TripSub(Node):
    def __init__(self):
        super().__init__('service_moving')
        self.create_subscription(Int32MultiArray, 'dynamixel', self.callback, 10)
        self.current = 0  # Initialize current

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
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(f'ID={dxl_id} Failed to change operating mode: {packetHandler.getTxRxResult(dxl_comm_result)}')
                raise Exception(f'ID={dxl_id} Failed to change operating mode')
            else:
                self.get_logger().info(f'ID={dxl_id} Operating mode changed to extended position control mode.')

            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(f'ID={dxl_id} Failed to enable torque: {packetHandler.getTxRxResult(dxl_comm_result)}')
                raise Exception(f'ID={dxl_id} Failed to enable torque')
            else:
                self.get_logger().info(f'ID={dxl_id} Dynamixel has been successfully connected')

    def callback(self, msg):
        # Write Goal Position
        goal_position1 = int((msg.data[0] + 335) / 0.088)
        goal_position2 = int((msg.data[1] + 25) / 0.088)

        # Set goal positions
        for dxl_id, goal_position in zip([DXL1_ID, DXL2_ID], [goal_position1, goal_position2]):
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, goal_position)
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(f'Failed to set goal position ID{dxl_id}: {packetHandler.getTxRxResult(dxl_comm_result)}')
            else:
                self.get_logger().info(f'ID{dxl_id} goal position: {round(goal_position)}')

        # Read present positions
        for dxl_id, offset in zip([DXL1_ID, DXL2_ID], [340, 212]):
            dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION)
            if dxl_comm_result == COMM_SUCCESS:
                self.get_logger().info(f'ID{dxl_id} ENCODER: {dxl_present_position * 0.088 - offset}')
            else:
                self.get_logger().error(f'Failed to read present position ID{dxl_id}: {packetHandler.getTxRxResult(dxl_comm_result)}')

    #def grip_callback(self, msg):
    #GRIPPER 작동 시작
        self.get_logger().info(f'Received grip command: {msg.data[2]}')
        
       

        if msg.data[2] == 1:  # Gripper 동작 명령 받은 경우
            self.gripper(int(70/ 0.088))  # 물체 잡기 시작
            time.sleep(1)
            self.torque()  # Current 읽는 함수 불러오기
             

            if self.current >= 100:  # 전류값 임계치 이상인지 확인
                self.get_logger().info('SUCCESS')
            else:  # 전류값 임계치 이하인지 확인
                self.gripper(int(200 / 0.088))  # GRIPPER 다시 원래대로 돌리기
                self.get_logger().info('FAIL')
        else:  # Gripper 동작 명령 안 받은 경우
            self.gripper(int(200 / 0.088))
            self.get_logger().info('INPUT GOGO')

    def gripper(self, goal_position):  # Gripper 제어 프로토콜 부분
        # ID=3 WRITE
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_GOAL_POSITION, goal_position)
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Failed to set goal position ID6: {packetHandler.getTxRxResult(dxl_comm_result)}')
        else:
            self.get_logger().info(f'ID6 goal position: {round(goal_position)}')

        # GRIPPER READ
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL3_ID, ADDR_PRESENT_POSITION)
        if dxl_comm_result == COMM_SUCCESS:
            self.get_logger().info(f'ID6 ENCODER: {int(dxl_present_position * 0.088)}')
        else:
            self.get_logger().error(f'Failed to read present position ID3: {packetHandler.getTxRxResult(dxl_comm_result)}')

    def torque(self):  # Gripper의 전류 읽어오는 함수
        dxl_present_current, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL3_ID, ADDR_PRESENT_CURRENT)
        
        if dxl_comm_result == COMM_SUCCESS:
            if dxl_present_current >= 65500 / 2 :
                self.current = abs(65500 - dxl_present_current)
            else:
                self.current = dxl_present_current

            self.get_logger().info(f'Present current: {self.current}')
        else:
            self.get_logger().error(f'Failed to read present current (Error: {dxl_error})')

        time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    dynamixel_subscriber = TripSub()
    try:
        rclpy.spin(dynamixel_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        dynamixel_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

