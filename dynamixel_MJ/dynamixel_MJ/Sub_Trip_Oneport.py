#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from .robotis_def import *
from .protocol2_packet_handler import * 
from .packet_handler import * 
from .port_handler import * 
from std_msgs.msg import Int32MultiArray


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


# Default setting
DXL1_ID                     = 4                 # Dynamixel#1 ID : 1
DXL2_ID                     = 5                 # Dynamixel#2 ID : 2
DXL3_ID                     = 6  

 
#DEVICENAME1                 = '/dev/ttyUSB3'
DEVICENAME1 = '/dev/ttyRS485'   

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold


# Initialize PortHandler instance
# Set the port path

portHandler = PortHandler(DEVICENAME1)


# Initialize PacketHandler instance

packetHandler = PacketHandler(PROTOCOL_VERSION)




class TripSub(Node):
    def __init__(self):
        super().__init__('dynamixel_subscriber')
        self.create_subscription(Int32MultiArray, 'dynamixel_double', self.callback, 10)
        self.current = 0
        
        #self.create_subscription(Int32MultiArray, 'joint_dy', self.callback, 10)
        
        
        #OPEN PORT

        if portHandler.openPort(): #ID = 1
            self.get_logger().info('Succeeded to open the port1')
        else:
            self.get_logger().error('Failed to open the port1')
            raise Exception('Failed to open the port1')
 


        # BAUD RATE
        if portHandler.setBaudRate(BAUDRATE): #ID = 1
            self.get_logger().info('Succeeded to change the baudrate1')
        else:
            self.get_logger().error('Failed to change the baudrate1')
            raise Exception('Failed to change the baudrate1')




       #Enable Torque 
       #ID = 1
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_OPERATING_MODE, EXT_POSITION_CONTROL_MODE)
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'ID=1 Failed to change operating mode: {packetHandler.getTxRxResult(dxl_comm_result)}')
            raise Exception('ID=1 Failed to change operating mode')
        else:
            self.get_logger().info('ID=1 Operating mode changed to extended position control mode.')
        
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'ID=1 Failed to enable torque: {packetHandler.getTxRxResult(dxl_comm_result)}')
            raise Exception('ID=1 Failed to enable torque')
        else:
            self.get_logger().info('ID=1 Dynamixel has been successfully connected')
 
        #ID = 2
        dxl2_comm_result, dxl2_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_OPERATING_MODE, EXT_POSITION_CONTROL_MODE)
        if dxl2_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'ID=2 Failed to change operating mode: {packetHandler.getTxRxResult(dxl2_comm_result)}')
            raise Exception('ID=2 Failed to change operating mode')
        else:
            self.get_logger().info('ID=2 Operating mode changed to extended position control mode.')
        
        dxl2_comm_result, dxl2_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl2_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'ID=2 Failed to enable torque: {packetHandler.getTxRxResult(dxl2_comm_result)}')
            raise Exception('ID=2 Failed to enable torque')
        else:
            self.get_logger().info('ID=2 Dynamixel has been successfully connected')

        #ID = 3
        dxl3_comm_result, dxl3_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_OPERATING_MODE, EXT_POSITION_CONTROL_MODE)
        if dxl3_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'ID=3 Failed to change operating mode: {packetHandler.getTxRxResult(dxl3_comm_result)}')
            raise Exception('ID=3 Failed to change operating mode')
        else:
            self.get_logger().info('ID=3 Operating mode changed to extended position control mode.')
        
        dxl3_comm_result, dxl3_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl3_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'ID=3 Failed to enable torque: {packetHandler.getTxRxResult(dxl3_comm_result)}')
            raise Exception('ID=3 Failed to enable torque')
        else:
            self.get_logger().info('ID=3 Dynamixel has been successfully connected')


    def callback(self, msg):
        #Write Goal Position

        goal_position1 = int((msg.data[0]+160)/0.088)
        goal_position2 = int((msg.data[1]+120)/0.088)
        goal_position3 = int((msg.data[2]) / 0.088)


        #ID = 1  

        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_POSITION, goal_position1)
        if dxl_comm_result != COMM_SUCCESS:
        
            self.get_logger().error(f'Failed to set goal position ID1: {packetHandler.getTxRxResult(dxl_comm_result)}')

        else:
            goal_position1 = round(goal_position1)
            self.get_logger().info(f'ID1 goal position: {goal_position1}')


        #ID = 2    


        dxl2_comm_result, dxl2_error = packetHandler.write4ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_POSITION, goal_position2)
        if dxl2_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Failed to set goal position ID2: {packetHandler.getTxRxResult(dxl2_comm_result)}')
        else:
            goal_position2 = round(goal_position2)
            self.get_logger().info(f'ID2 goal position: {goal_position2}')

        #ID = 3    

        dxl3_comm_result, dxl3_error = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_GOAL_POSITION, goal_position3)
        if dxl3_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Failed to set goal position ID3: {packetHandler.getTxRxResult(dxl3_comm_result)}')
            
        else:
            goal_position3 = round(goal_position3)
            self.get_logger().info(f'ID3 goal position: {goal_position3}')
            

        #ID=1 
        dxl1_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL1_ID, ADDR_PRESENT_POSITION)
        if dxl_comm_result == COMM_SUCCESS:
            #dxl1_present_position = -dxl1_present_position
            self.get_logger().info(f'ID1 ENCODER: {dxl1_present_position*0.088}')

        else:
            self.get_logger().error(f'Failed to read present position: {packetHandler.getTxRxResult(dxl_comm_result)}')

            #ID=2 
        dxl2_present_position, dxl2_comm_result, dxl2_error = packetHandler.read4ByteTxRx(portHandler, DXL2_ID, ADDR_PRESENT_POSITION)
        if dxl2_comm_result == COMM_SUCCESS:

            #dxl2_present_position = -dxl2_present_position
            self.get_logger().info(f'ID2 ENCODER: {dxl2_present_position*0.088}')
         
        else:
            self.get_logger().error(f'Failed to read present position: {packetHandler.getTxRxResult(dxl2_comm_result)}')

            #ID=3 
        dxl3_present_position, dxl3_comm_result, dxl3_error = packetHandler.read4ByteTxRx(portHandler, DXL3_ID, ADDR_PRESENT_POSITION)
        if dxl3_comm_result == COMM_SUCCESS:
            #dxl3_present_position = -dxl3_present_position
            self.get_logger().info(f'ID3 ENCODER: {dxl3_present_position*0.088}')    
            #print(f'Present current: {self.current}')
        else:
            self.get_logger().error(f'Failed to read present position: {packetHandler.getTxRxResult(dxl3_comm_result)}')
            
       #ID3 Current
        dxl3_present_current, dxl3_comm_result, dxl3_error = packetHandler.read4ByteTxRx(
            portHandler, DXL3_ID, ADDR_PRESENT_CURRENT
        )
        if dxl_comm_result == COMM_SUCCESS:
            #self.current = (0xFFFFFFFF ^ dxl3_present_current) + 1

            #self.current = 0.00269* (dxl3_present_current/65536 - 65536)
            self.current_init = (dxl3_present_current & 0XFFFF)
            #print(self.current)
            if self.current_init >= 65536/2:
                #self.current =  0.00269* (dxl3_present_current/65536 - 65536) - 4280000000
                self.current = (65536 - self.current_init)*(-1)
                print(f'Present current: {self.current}')
            else:
                self.current = self.current_init
                #print(f'Present current: {self.current}')
          
            #if(dxl3_present_current >= 65535/2):
            #     dxl3_present_current = abs(65535 - dxl3_present_current)
            #     print(f'Present current: {dxl3_present_current}')
            # else:
                print(f'Present current: {self.current}')

        else:
            print(f'Failed to read present current (Error: {dxl3_error})')

def main(args=None):
    rclpy.init(args=args)
    dynamixel_subscriber = TripSub()
    rclpy.spin(dynamixel_subscriber)
    dynamixel_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


