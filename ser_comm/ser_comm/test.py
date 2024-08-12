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

#from .serial_comm import *

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

#0~3 joint
from custom_interfaces.srv import Protocool
import time as t
from struct import pack



class TripSub(Node):
    def __init__(self):
        super().__init__('service_moving')
        self.create_subscription(Int32MultiArray, 'joint_dy', self.callback, 10)
        self.srv = self.create_service(SetBool, 'ActiveGripper', self.grip_callback)
        self.srv_yp = self.create_service(Protocool, 'command', self.custom_service_callback)
        
        ### init ###

        self.ser_comm_list = []
        self.ser_comm_byte = []


        ############
        
        
        
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
            print("COUNT1")
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(f'ID={dxl_id} Failed to change operating mode: {packetHandler.getTxRxResult(dxl_comm_result)}')
                raise Exception(f'ID={dxl_id} Failed to change operating mode')
            else:
                self.get_logger().info(f'ID={dxl_id} Operating mode changed to extended position control mode.')

            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            print("COUNT2")
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(f'ID={dxl_id} Failed to enable torque: {packetHandler.getTxRxResult(dxl_comm_result)}')
                raise Exception(f'ID={dxl_id} Failed to enable torque')
            else:
                self.get_logger().info(f'ID={dxl_id} Dynamixel has been successfully connected')


        # Read present positions
        for dxl_id, offset in zip([DXL1_ID, DXL2_ID], [348, 25]):
            dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION)
            if dxl_comm_result == COMM_SUCCESS:
                self.get_logger().info(f'ID{dxl_id} ENCODER: {dxl_present_position * 0.088 - offset}')
            else:
                self.get_logger().error(f'Failed to read present position ID{dxl_id}: {packetHandler.getTxRxResult(dxl_comm_result)}')


    def custom_service_callback(self, request, response):
        # self.get_logger().info(f'received : {request.sercommand} , {request.codecommand}')
        self.ser_comm_list.append(request.sercommand)
        self.ser_comm_byte = bytes(request.sercommand)
        # self.get_logger().info(f'init ser_comm_list {len(self.ser_comm_list)}')

        if self.ser_comm_byte != b'':
            #portHandler.ser.write(self.ser_comm_byte)
            # print("COUNT5")
            # print a pretty message
            # self.get_logger().info(f'send : {self.ser_comm_byte}')
            # response state
            response.success = True
        elif request.codecommand == "Init":
            self.get_logger().info('Initializing!!')
            self.nuri_init()
            # response state
            response.success = True
        elif request.codecommand == "Read":
            self.get_logger().info('waiting to read!!')
            while(1):
                if portHandler.ser.readable():
                    response.feedback = list(self.ser.readline())
                    break

        else:
            # response state
            response.success = False
        
        # return the response parameter
        return response
    

    def nuri_init(self):
        for i in range(4):
            self.get_logger().info(f'{i} motor initializing...')
            portHandler.ser.write(call_feedback(i, 0xA0))
            #print("COUNT6")
            count = 0
            while 1:
                if portHandler.ser.readable():
                    readdata = portHandler.ser.readline()
                    id = self.checkID(readdata)
                    if id == i:
                        self.get_logger().info(f'{i} motor initialized!')
                        break
                    elif id == -1:
                        if count == 3:
                            self.get_logger().error(f'{i} motor failed!!')
                            break
                        else:
                            self.get_logger().warn('retry...')
                            portHandler.ser.write(call_feedback(i, 0xA0))
                            print("COUNT7")
                            count = count + 1
                    else:
                        self.get_logger().error(f'responsed but not {i} motor. retry...')
                        portHandler.ser.write(call_feedback(i, 0xA0))
                        print("COUNT8")
                        count = count + 1
    def checkID(self, data):
        readdataHEX = data.hex()
        try :
            id = int(readdataHEX[4:6])
            return id
        except:
            self.get_logger().warn('empty data receved.')
            return -1
    
    def callback(self, msg):
        # Write Goal Position
        goal_position1 = int((msg.data[0] + 347.488) / 0.088)
        goal_position2 = int((msg.data[1] + 25) / 0.088)

        if (len(self.ser_comm_list)>0) :
            for i in range(len(self.ser_comm_list)) : 
                self.get_logger().info(f'{self.ser_comm_list[0]}')
                portHandler.ser.write(bytes(self.ser_comm_list[0]))
                self.ser_comm_list.pop(0)
        else :
            pass

        # Set goal positions
        for dxl_id, goal_position in zip([DXL1_ID, DXL2_ID], [goal_position1, goal_position2]):
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, goal_position)
            # print("COUNT3")
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(f'Failed to set goal position ID{dxl_id}: {packetHandler.getTxRxResult(dxl_comm_result)}')
            else:
                # self.get_logger().info(f'ID{dxl_id} goal position: {round(goal_position)}')
                pass



    def grip_callback(self, request, response):
        grip = request.data
        # self.get_logger().info(f'Received grip command: {grip}')
        
        

        if grip: # Gripper 동작 명령받은 경우
            self.gripper(int(70 / 0.088)) # 물체 잡기 시작
            self.torque()  # Current읽는 함수 불러오기

            if self.current >= 150:  #전류값 임계치 이상인지 확인
                response.success = True
                response.message = 'Grip success'

            else: #전류값 임계치 이하인지 확인
                response.success = False
                response.message = 'Grip failed'
                self.gripper(int(200 / 0.088)) # GRIPPER 다시 원래대로 돌리기

        else: # Gripper 동작 명령 안 받은 경우
            self.gripper(int(200 / 0.088))
            self.get_logger().info('INPUT GOGO')

        return response


    def gripper(self, goal_position): #Gripper 제어 프로토콜 부분
        # ID=3 WRITE
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL3_ID, ADDR_GOAL_POSITION, goal_position)
        print("COUNT4")
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


    def torque(self): #Gripper의 전류 읽어오는 함수
        dxl_present_current, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL3_ID, ADDR_PRESENT_CURRENT)
        
        if dxl_comm_result == COMM_SUCCESS:
            if dxl_present_current >= int(65533 / 2):
                self.current = abs(65533 - dxl_present_current)
            else:
                self.current = dxl_present_current

            self.get_logger().info(f'Present current: {self.current}')

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



#0~3 joint
def call_feedback(Id, Mode):
    motor_id = format(Id, '#04x')
    data_num = '0x02'
    mode = format(Mode, '#04x')
    data_array = [motor_id, data_num, mode]
    return attach_checksum(data_array)


def attach_checksum(data_arr):
    sum = 0x00
    tem_arr = data_arr
    for i in data_arr:
        sum = sum + int(i, 16)
    sumByte = pack('>i', sum)
    checksum = sumByte[-1] ^ 0xff
    tem_arr.insert(2, format(checksum, '#04x'))
    return protocool_comm(tem_arr)

    
def protocool_comm(dataWithChecksum):
    command = bytes.fromhex('FFFE')
    for i in dataWithChecksum:
        #print('-----------------')
        #print(i)
        command = command + bytes.fromhex(i[2:])
        #print('-------')
        #print(command)
    return command