import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

#Protocol을 위해 Import 할 파일
from .robotis_def import *
from .protocol2_packet_handler import * 
from .packet_handler import * 
from .port_handler import * 

import time

ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

#DXL_MINIMUM_POSITION_VALUE = 0
#DXL_MAXIMUM_POSITION_VALUE = 4095

BAUDRATE = 57600
ADDR_PRESENT_CURRENT = 126
PROTOCOL_VERSION = 2.0
DXL_ID = 4
#DEVICENAME = '/dev/ttyUSB3'
DEVICENAME = '/dev/ttyRS485'   

TORQUE_ENABLE = 1    

class DynamixelSubscriber(Node):
    def __init__(self):
        super().__init__('dynamixel_control')

        self.declare_parameter('devicename', DEVICENAME)
        self.declare_parameter('baudrate', BAUDRATE)
        self.declare_parameter('dxl_id', DXL_ID)

        devicename = self.get_parameter('devicename').value
        baudrate = self.get_parameter('baudrate').value
        dxl_id = self.get_parameter('dxl_id').value

        self.portHandler = PortHandler(devicename)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self.dxl_id = dxl_id
        #self.dxl_goal_position = DXL_MINIMUM_POSITION_VALUE  

        if self.portHandler.openPort():
            self.get_logger().info(f'Succeeded to open the port ({devicename})')
        else:
            self.get_logger().error('Failed to open the port')
            rclpy.shutdown()

        if self.portHandler.setBaudRate(baudrate):
            self.get_logger().info(f'Succeeded to change the baudrate ({baudrate})')
        else:
            self.get_logger().error('Failed to change the baudrate')
            rclpy.shutdown()

        self.enable_dynamixel()

        self.subscription = self.create_subscription(
            Int32,
            'dynamixel_goal_position',
            self.goal_position_callback,
            10
        )

        self.timer = self.create_timer(1.0, self.read_present_position)  

    def enable_dynamixel(self):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, self.dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE 
        )
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Failed to enable Dynamixel (Error: {dxl_error})')
            rclpy.shutdown()

    def goal_position_callback(self, msg):
        self.dxl_goal_position = int((msg.data)/ 0.088)
        self.get_logger().info(f'Received goal position: {self.dxl_goal_position}')
        self.set_goal_position(self.dxl_goal_position)
        
        #if msg.data > 0:
         #   self.set_goal_position(-self.dxl_goal_position)
          #  time.sleep(3)
           # self.set_goal_position(self.dxl_goal_position)

    def set_goal_position(self, goal_position):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler, self.dxl_id, ADDR_GOAL_POSITION, goal_position
        )
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Failed to set goal position (Error: {dxl_error})')

    def read_present_position(self):
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(
            self.portHandler, self.dxl_id, ADDR_PRESENT_POSITION
        )
        if dxl_comm_result == COMM_SUCCESS:
          
            self.get_logger().info(f'Present position: {dxl_present_position*0.088}')
        else:
            self.get_logger().error(f'Failed to read present position (Error: {dxl_error})')
        #전류 확인
        dxl_present_current, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(
            self.portHandler, self.dxl_id, ADDR_PRESENT_CURRENT
        )

        if dxl_comm_result == COMM_SUCCESS:
              self.current_init = (dxl_present_current & 0XFFFF)
          
              if(self.current_init >= 65536/2):
                 self.current  = (65536 - self.current_init*(-1))
                 self.get_logger().error(f'Present current: {self.current }')
              else:
                  self.current = self.current_init
                  self.get_logger().error(f'Present current: {self.current }')

        else:
            self.get_logger().error(f'Failed to read present current (Error: {dxl_error})')

        #time.sleep(1)




    def run(self):
        rclpy.spin(self)

        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, self.dxl_id, ADDR_TORQUE_ENABLE, 0
        )
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Failed to disable Dynamixel (Error: {dxl_error})')

        self.portHandler.closePort()
        self.get_logger().info('Closed port')

def main(args=None):
    rclpy.init(args=args)
    node = DynamixelSubscriber()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

