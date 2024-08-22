#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from dynamixel_sdk import * 
from std_msgs.msg import Float32MultiArray

# Control table address
ADDR_OPERATING_MODE         = 11              
ADDR_TORQUE_ENABLE          = 64               
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_POSITION       = 132

# Protocol version
PROTOCOL_VERSION            = 2.0            
BAUDRATE                    = 57600   

# Default setting
         
MAX_POSITION_VALUE          = 1048575             
EXT_POSITION_CONTROL_MODE   = 4                 

ESC_ASCII_VALUE             = 0x1b
SPACE_ASCII_VALUE           = 0x20


# Default setting
DXL1_ID                     = 1                 # Dynamixel#1 ID : 1
DXL2_ID                     = 2                 # Dynamixel#2 ID : 2
DXL3_ID                     = 3  

DEVICENAME2                 = '/dev/ttyUSB0'    
DEVICENAME1                 = '/dev/ttyUSB1'   

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold


# Initialize PortHandler instance
# Set the port path

portHandler1 = PortHandler(DEVICENAME1)
portHandler2 = PortHandler(DEVICENAME2)

# Initialize PacketHandler instance

packetHandler = PacketHandler(PROTOCOL_VERSION)



class TripSub(Node):
    def __init__(self):
        super().__init__('dynamixel_subscriber')
        self.create_subscription(Float32MultiArray, 'dynamixel_double', self.callback, 10)
        
        #OPEN PORT

        if portHandler1.openPort(): #ID = 1
            self.get_logger().info('Succeeded to open the port1')
        else:
            self.get_logger().error('Failed to open the port1')
            raise Exception('Failed to open the port1')

        if portHandler2.openPort(): #ID = 2
            self.get_logger().info('Succeeded to open the port2')
        else:
            self.get_logger().error('Failed to open the port2')
            raise Exception('Failed to open the port2')       


        # BAUD RATE
        if portHandler1.setBaudRate(BAUDRATE): #ID = 1
            self.get_logger().info('Succeeded to change the baudrate1')
        else:
            self.get_logger().error('Failed to change the baudrate1')
            raise Exception('Failed to change the baudrate1')

        if portHandler2.setBaudRate(BAUDRATE): #ID = 2
            self.get_logger().info('Succeeded to change the baudrate2')
        else:
            self.get_logger().error('Failed to change the baudrate2')
            raise Exception('Failed to change the baudrate2')
        



       #Enable Torque 
       #ID = 1
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler1, DXL1_ID, ADDR_OPERATING_MODE, EXT_POSITION_CONTROL_MODE)
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'ID=1 Failed to change operating mode: {packetHandler.getTxRxResult(dxl_comm_result)}')
            raise Exception('ID=1 Failed to change operating mode')
        else:
            self.get_logger().info('ID=1 Operating mode changed to extended position control mode.')
        
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler1, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'ID=1 Failed to enable torque: {packetHandler.getTxRxResult(dxl_comm_result)}')
            raise Exception('ID=1 Failed to enable torque')
        else:
            self.get_logger().info('ID=1 Dynamixel has been successfully connected')
 
        #ID = 2
        dxl2_comm_result, dxl2_error = packetHandler.write1ByteTxRx(portHandler2, DXL2_ID, ADDR_OPERATING_MODE, EXT_POSITION_CONTROL_MODE)
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'ID=2 Failed to change operating mode: {packetHandler.getTxRxResult(dxl_comm_result)}')
            raise Exception('ID=2 Failed to change operating mode')
        else:
            self.get_logger().info('ID=2 Operating mode changed to extended position control mode.')
        
        dxl2_comm_result, dxl2_error = packetHandler.write1ByteTxRx(portHandler2, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'ID=2 Failed to enable torque: {packetHandler.getTxRxResult(dxl_comm_result)}')
            raise Exception('ID=2 Failed to enable torque')
        else:
            self.get_logger().info('ID=2 Dynamixel has been successfully connected')

        #ID = 3
        dxl3_comm_result, dxl3_error = packetHandler.write1ByteTxRx(portHandler2, DXL3_ID, ADDR_OPERATING_MODE, EXT_POSITION_CONTROL_MODE)
        if dxl3_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'ID=3 Failed to change operating mode: {packetHandler.getTxRxResult(dxl_comm_result)}')
            raise Exception('ID=3 Failed to change operating mode')
        else:
            self.get_logger().info('ID=3 Operating mode changed to extended position control mode.')
        
        dxl3_comm_result, dxl3_error = packetHandler.write1ByteTxRx(portHandler2, DXL3_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl3_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'ID=3 Failed to enable torque: {packetHandler.getTxRxResult(dxl_comm_result)}')
            raise Exception('ID=3 Failed to enable torque')
        else:
            self.get_logger().info('ID=3 Dynamixel has been successfully connected')


    def callback(self, msg):
        #Write Goal Position

        goal_position1 = int(msg.data[0])
        goal_position2 = int(msg.data[1])
        goal_position3 = int(msg.data[2])


        #ID = 1  

        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler1, DXL1_ID, ADDR_GOAL_POSITION, goal_position1)
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Failed to set goal position ID1: {packetHandler.getTxRxResult(dxl_comm_result)}')
        else:
            self.get_logger().info(f'Set goal position to ID1: {goal_position1}')


        #ID = 2    

        dxl2_comm_result, dxl2_error = packetHandler.write4ByteTxRx(portHandler2, DXL2_ID, ADDR_GOAL_POSITION, goal_position2)
        if dxl2_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Failed to set goal position ID2: {packetHandler.getTxRxResult(dxl2_comm_result)}')
        else:
            self.get_logger().info(f'Set goal position to ID2: {goal_position2}')

        #ID = 3    

        dxl3_comm_result, dxl3_error = packetHandler.write4ByteTxRx(portHandler2, DXL3_ID, ADDR_GOAL_POSITION, goal_position3)
        if dxl3_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Failed to set goal position ID3: {packetHandler.getTxRxResult(dxl3_comm_result)}')
        else:
            self.get_logger().info(f'Set goal position to ID3: {goal_position3}')


        while rclpy.ok():
            #Read Present Position

            #ID=1 
            dxl1_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler1, DXL1_ID, ADDR_PRESENT_POSITION)
            if dxl_comm_result == COMM_SUCCESS:
                self.get_logger().info(f'Present position: {dxl1_present_position}')
                break
            else:
                self.get_logger().error(f'Failed to read present position: {packetHandler.getTxRxResult(dxl_comm_result)}')

            #ID=2 
            dxl2_present_position, dxl2_comm_result, dxl2_error = packetHandler.read4ByteTxRx(portHandler2, DXL2_ID, ADDR_PRESENT_POSITION)
            if dxl2_comm_result == COMM_SUCCESS:
                self.get_logger().info(f'Present position: {dxl2_present_position}')
                break
            else:
                self.get_logger().error(f'Failed to read present position: {packetHandler.getTxRxResult(dxl2_comm_result)}')


            #ID=3 
            dxl3_present_position, dxl3_comm_result, dxl3_error = packetHandler.read4ByteTxRx(portHandler2, DXL3_ID, ADDR_PRESENT_POSITION)
            if dxl3_comm_result == COMM_SUCCESS:
                self.get_logger().info(f'Present position: {dxl3_present_position}')
                break
            else:
                self.get_logger().error(f'Failed to read present position: {packetHandler.getTxRxResult(dxl3_comm_result)}')

def main(args=None):
    rclpy.init(args=args)
    dynamixel_subscriber = TripSub()
    rclpy.spin(dynamixel_subscriber)
    dynamixel_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

