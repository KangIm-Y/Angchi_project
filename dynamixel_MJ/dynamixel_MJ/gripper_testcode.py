#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from .robotis_def import *
from .protocol2_packet_handler import * 
from .packet_handler import * 
from .port_handler import * 
import time as t

# Control table address
ADDR_OPERATING_MODE = 11              
ADDR_TORQUE_ENABLE = 64               
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_CURRENT = 126

# Protocol version
PROTOCOL_VERSION = 2.0            
BAUDRATE = 57600

# Default setting
MAX_POSITION_VALUE = 1048575             
EXT_POSITION_CONTROL_MODE = 4                 

# Dynamixel IDs
DXL1_ID = 4
DXL2_ID = 5
DXL3_ID = 6  

#DEVICENAME = '/dev/ttyUSB3'
DEVICENAME = '/dev/ttyRS485'   

TORQUE_ENABLE = 1                 
TORQUE_DISABLE = 0                 
DXL_MOVING_STATUS_THRESHOLD = 20                

# Initialize PortHandler instance
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
packetHandler = PacketHandler(PROTOCOL_VERSION)

# 4~5 joint
OFFSET_4 = 160
OFFSET_5 = 120

class TripSub(Node):
    def __init__(self):
        super().__init__('dynamixel_subscriber')
        
        # Create subscription to Int32MultiArray messages
        self.create_subscription(Int32MultiArray, 'dynamixel_double', self.callback, 10)

        self.current = 0

        self.initialize_port()

    def initialize_port(self):
        """Open the port, set the baud rate, enable torque, and flush the serial buffer."""
        try:
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

            # Flush serial buffers
            portHandler.ser.flushInput()
            #portHandler.ser.flushOutput()

            for dxl_id in [DXL1_ID, DXL2_ID, DXL3_ID]:
                # Change operating mode to extended position control mode
                dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE, EXT_POSITION_CONTROL_MODE)
                if dxl_comm_result != COMM_SUCCESS:
                    self.get_logger().error(f'ID={dxl_id} Failed to change operating mode: {packetHandler.getTxRxResult(dxl_comm_result)}')
                    raise Exception(f'ID={dxl_id} Failed to change operating mode')
                else:
                    self.get_logger().info(f'ID={dxl_id} Operating mode changed to extended position control mode.')

                # Enable torque
                dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
                if dxl_comm_result != COMM_SUCCESS:
                    self.get_logger().error(f'ID={dxl_id} Failed to enable torque: {packetHandler.getTxRxResult(dxl_comm_result)}')
                    raise Exception(f'ID={dxl_id} Failed to enable torque')
                else:
                    self.get_logger().info(f'ID={dxl_id} Dynamixel torque enabled successfully')

            for dxl_id, offset in zip([DXL1_ID, DXL2_ID], [OFFSET_4, OFFSET_5]):
                dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION)
                if dxl_comm_result == COMM_SUCCESS:
                    self.get_logger().info(f'ID{dxl_id} ENCODER: {dxl_present_position * 0.088 - offset}')
                else:
                    self.get_logger().error(f'Failed to read present position ID{dxl_id}: {packetHandler.getTxRxResult(dxl_comm_result)}')

        except Exception as e:
            self.get_logger().error(f'Error initializing port: {e}')

    def check_port_status(self):
        """Check if the port is open and functional."""
        if not portHandler.ser.is_open:
            self.get_logger().error('Serial port is not open.')
            return False
        try:
            portHandler.ser.write(b'')  # Try sending a test command
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to write to port: {e}')
            return False

    def reinitialize_port(self):
        """Reinitialize the serial port."""
        try:
            portHandler.closePort()  # Close the port if it's open
            t.sleep(0.5)  # Wait for a moment before reopening
            self.initialize_port()  # Reinitialize the port
            self.get_logger().info('Serial port reinitialized successfully.')
        except Exception as e:
            self.get_logger().error(f'Failed to reinitialize the port: {e}')

    def callback(self, msg):
        if not self.check_port_status():
            self.get_logger().error('Port check failed. Reinitializing port...')
            self.reinitialize_port()
            if not self.check_port_status():
                self.get_logger().error('Port reinitialization failed. Skipping callback processing.')
                return
        
        goal_position1 = int((-msg.data[0] + OFFSET_4) / 0.088)
        goal_position2 = int((msg.data[1] + OFFSET_5) / 0.088)

        for dxl_id, goal_position in zip([DXL1_ID, DXL2_ID], [goal_position1, goal_position2]):
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, goal_position)
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(f'Failed to set goal position ID{dxl_id}: {packetHandler.getTxRxResult(dxl_comm_result)}')

        if msg.data[2] == 1:
            self.get_logger().info('Gripper action requested')
            self.perform_grip_action()
        else:
            self.get_logger().info('No gripper action requested')

    def perform_grip_action(self):
        angle_step = 33
        initial_position = int(360 / 0.088)
        final_position = int(150 / 0.088)
        
        self.gripper(initial_position)
        for goal_position in range(int(initial_position), int(final_position), -angle_step):
            self.gripper(goal_position)
            self.get_logger().info('-------------------------Gripping-------------------------')
            self.torque()  # Update self.current
            t.sleep(0.01)

        self.get_logger().info('-------------------------Finish GRIP-------------------------')

        if self.current <= -100:
            self.get_logger().info(f'Present current: {self.current} - Grip success')
        else:
            self.get_logger().info(f'Present current: {self.current} - Grip failed')
            self.gripper(initial_position)

    def gripper(self, goal_position):
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_GOAL_POSITION, goal_position)
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Failed to set goal position ID3: {packetHandler.getTxRxResult(dxl_comm_result)}')
        else:
            self.get_logger().info(f'ID3 goal position: {round(goal_position)}')

        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL3_ID, ADDR_PRESENT_POSITION)
        if dxl_comm_result == COMM_SUCCESS:
            self.get_logger().info(f'ID3 ENCODER: {dxl_present_position * 0.088}')
        else:
            self.get_logger().error(f'Failed to read present position ID3: {packetHandler.getTxRxResult(dxl_comm_result)}')

    def torque(self):
        dxl_present_current, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL3_ID, ADDR_PRESENT_CURRENT)
        
        if dxl_comm_result == COMM_SUCCESS:
            self.current_init = (dxl_present_current & 0XFFFF)
            if self.current_init >= 65536 / 2:
                self.current = (65536 - self.current_init) * (-1)
            else:
                self.current = self.current_init

            self.get_logger().info(f'Present current: {self.current}')
        else:
            self.get_logger().error(f'Failed to read present current (Error: {dxl_error})')

def main(args=None):
    rclpy.init(args=args)

    while True:
        dynamixel_subscriber = TripSub()

        try:
            while rclpy.ok():
                if not dynamixel_subscriber.check_port_status():
                    dynamixel_subscriber.get_logger().error('Serial port disconnected. Restarting node...')
                    dynamixel_subscriber.destroy_node()
                    rclpy.shutdown()
                    t.sleep(0.5)  # Shorter wait time before restarting
                    break  # Exit the inner loop to restart the node

                rclpy.spin_once(dynamixel_subscriber)

        except KeyboardInterrupt:
            dynamixel_subscriber.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            dynamixel_subscriber.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
