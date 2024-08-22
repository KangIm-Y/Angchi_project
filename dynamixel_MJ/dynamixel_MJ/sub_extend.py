#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
#from dynamixel_sdk import * 
from std_msgs.msg import Int32


from .robotis_def import *
from .protocol2_packet_handler import * 
from .packet_handler import * 
from .port_handler import * 




# Control table address
ADDR_OPERATING_MODE         = 11              
ADDR_TORQUE_ENABLE          = 64               
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_POSITION       = 132

# Protocol version
PROTOCOL_VERSION            = 2.0            

# Default setting
DXL_ID                      = 6                 
BAUDRATE                    = 57600             
DEVICENAME                  = '/dev/ttyUSB0'   
                                               

TORQUE_ENABLE               = 1                
TORQUE_DISABLE              = 0               
MAX_POSITION_VALUE          = 1048575
DXL_MOVING_STATUS_THRESHOLD = 20                
EXT_POSITION_CONTROL_MODE   = 4                 

ESC_ASCII_VALUE             = 0x1b
SPACE_ASCII_VALUE           = 0x20

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

class DynamixelSubscriber(Node):
    def __init__(self):
        super().__init__('dynamixel_subscriber')
        self.create_subscription(Int32, 'dynamixel_goal_position', self.callback, 10)
        if portHandler.openPort():
            self.get_logger().info('Succeeded to open the port')
            
        else:
            self.get_logger().error('Failed to open the port')
            raise Exception('Failed to open the port')
        
        if portHandler.setBaudRate(BAUDRATE):
            self.get_logger().info('Succeeded to change the baudrate')
        else:
            self.get_logger().error('Failed to change the baudrate')
            raise Exception('Failed to change the baudrate')
        
       
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, EXT_POSITION_CONTROL_MODE)
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Failed to change operating mode: {packetHandler.getTxRxResult(dxl_comm_result)}')
            raise Exception('Failed to change operating mode')
        else:
            self.get_logger().info('Operating mode changed to extended position control mode.')
        
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Failed to enable torque: {packetHandler.getTxRxResult(dxl_comm_result)}')
            raise Exception('Failed to enable torque')
        else:
            self.get_logger().info('Dynamixel has been successfully connected')

    def callback(self, msg):
        goal_position = int(msg.data)

        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, goal_position)
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Failed to set goal position: {packetHandler.getTxRxResult(dxl_comm_result)}')
        else:
            self.get_logger().info(f'Set goal position to: {goal_position}')

        while rclpy.ok():

            dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
            if dxl_comm_result == COMM_SUCCESS:
                self.get_logger().info(f'Present position: {dxl_present_position}')
                break
            else:
                self.get_logger().error(f'Failed to read present position: {packetHandler.getTxRxResult(dxl_comm_result)}')

def main(args=None):
    rclpy.init(args=args)
    dynamixel_subscriber = DynamixelSubscriber()
    rclpy.spin(dynamixel_subscriber)
    dynamixel_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

