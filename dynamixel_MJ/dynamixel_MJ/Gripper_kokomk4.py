#!/usr/bin/env python3


#4~6 joint
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

#DYNAMIXEL Protocol-> Import 할 파일
from .robotis_def import *
from .protocol2_packet_handler import * 
from .packet_handler import * 
from .port_handler import * 

#0~3 joint 
from custom_interfaces.srv import PositionService
from std_srvs.srv import SetBool

#0~3 joint
from custom_interfaces.srv import Protocool
import time as t
from struct import pack


# Control table address
ADDR_OPERATING_MODE         = 11              
ADDR_TORQUE_ENABLE          = 64               
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_POSITION       = 132
ADDR_PRESENT_CURRENT = 126
ADDR_PROFILE_VELOCITY = 112

# Protocol version
PROTOCOL_VERSION            = 2.0            
BAUDRATE                    = 57600


# Default setting
MAX_POSITION_VALUE          = 1048575             
EXT_POSITION_CONTROL_MODE   = 4                 
PROFILE_VELOCITY = 2000

ESC_ASCII_VALUE             = 0x1b
SPACE_ASCII_VALUE           = 0x20



# Dynamixel ID
DXL1_ID                     = 4
DXL2_ID                     = 5
DXL3_ID                     = 6  

DEVICENAME                 = '/dev/ttyRS485'  
#DEVICENAME = '/dev/ttyUSB3' 
TORQUE_ENABLE               = 1                 
TORQUE_DISABLE              = 0                 
DXL_MOVING_STATUS_THRESHOLD = 20                

# Initialize PortHandler instance
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
packetHandler = PacketHandler(PROTOCOL_VERSION)


#4~5 joint
OFFSET_4 = 160
OFFSET_5 = 120


datalen = 12

class TripSub(Node):
    def __init__(self):
        super().__init__('service_moving')

        #Subscriber
        self.create_subscription(Int32MultiArray, 'joint_dy', self.callback, 10)

        #Service
        self.srv = self.create_service(SetBool, 'ActiveGripper', self.grip_callback)
        self.srv_yp = self.create_service(Protocool, 'command', self.custom_service_callback)

        #Gripper Current
        self.current = 0

        ### protocol write buffer ###
        self.ser_comm_list = []
        self.ser_comm_byte = []


        
        
        #Dynamixel Initializing

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
        

        for dxl_id in [DXL1_ID, DXL2_ID, DXL3_ID]:

            #Operating Mode
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE, EXT_POSITION_CONTROL_MODE)
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(f'ID={dxl_id} Failed to change operating mode: {packetHandler.getTxRxResult(dxl_comm_result)}')
                raise Exception(f'ID={dxl_id} Failed to change operating mode')
            else:
                self.get_logger().info(f'ID={dxl_id} Operating mode changed to extended position control mode.')

            #Profile Velocity
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_PROFILE_VELOCITY, PROFILE_VELOCITY )
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(f'ID={dxl_id} Failed to Profile Velocity: {packetHandler.getTxRxResult(dxl_comm_result)}')
                raise Exception(f'ID={dxl_id} Failed to change Profile Velocity')
            else:
                self.get_logger().info(f'ID={dxl_id} Profile Velocity mode changed')


            #Torque Enable Mode
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(f'ID={dxl_id} Failed to enable torque: {packetHandler.getTxRxResult(dxl_comm_result)}')
                raise Exception(f'ID={dxl_id} Failed to enable torque')
            else:
                self.get_logger().info(f'ID={dxl_id} Dynamixel has been successfully connected')
        



        # Read present positions
        for dxl_id, offset in zip([DXL1_ID, DXL2_ID], [OFFSET_4, OFFSET_5]):
            dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION)
            if dxl_comm_result == COMM_SUCCESS:
                self.get_logger().info(f'ID{dxl_id} ENCODER: {dxl_present_position * 0.088 - offset}')
            else:
                self.get_logger().error(f'Failed to read present position ID{dxl_id}: {packetHandler.getTxRxResult(dxl_comm_result)}')



    def custom_service_callback(self, request, response):
        self.get_logger().info(f'received : {request.sercommand} , {request.codecommand}')

        if request.codecommand == "Move":
            try:

        #-----------------------------send command motor 0------------------------        
                data = bytes(request.sercommand.id0)
                if data != b'':
                    portHandler.ser.write(data)
                    # print a pretty message
                    self.get_logger().info(f'send m0 : {data}')
                    portHandler.ser.write(call_feedback(0, 0xA1))
                    while(1):
                        #self.get_logger().info('waiting to read m0')
                        if portHandler.ser.readable():
                            responsedatabytes = b''
                            while 1:
                                tmpdata = portHandler.ser.read()
                                if tmpdata != b'':
                                    responsedatabytes = tmpdata
                                    break
                            for i in range(datalen):
                                responsedatabytes = responsedatabytes + portHandler.ser.read()
                            responsedata = list(responsedatabytes)
                            response.feedback.id0 = responsedata
                            #self.get_logger().info(f"send data : {response.feedback.id0}")
                            break

        #-----------------------------send command motor 1------------------------  
                data = bytes(request.sercommand.id1)
                if data != b'':
                    portHandler.ser.write(data)
                    # print a pretty message
                    self.get_logger().info(f'send m1 : {data}')
                    portHandler.ser.write(call_feedback(1, 0xA1))
                    while(1):
                        #self.get_logger().info('waiting to read m1')
                        if portHandler.ser.readable():
                            #responsedatabytes = self.ser.readline()
                            responsedatabytes = b''
                            while 1:
                                tmpdata = portHandler.ser.read()
                                if tmpdata != b'':
                                    responsedatabytes = tmpdata
                                    break
                            for i in range(datalen):
                                responsedatabytes = responsedatabytes + portHandler.ser.read()
                            #self.get_logger().info(f"feedback data : {responsedatabytes}")
                            responsedata = list(responsedatabytes)
                            response.feedback.id1 = responsedata
                            #self.get_logger().info(f"send data : {response.feedback.id1}")
                            break

        #-----------------------------send command motor 2------------------------  
                data = bytes(request.sercommand.id2)
                if data != b'':
                    portHandler.ser.write(data)
                    # print a pretty message
                    self.get_logger().info(f'send m2 : {data}')
                    portHandler.ser.write(call_feedback(2, 0xA1))
                    while(1):
                        #self.get_logger().info('waiting to read m2')
                        if portHandler.ser.readable():
                            #responsedatabytes = self.ser.readline()
                            responsedatabytes = b''
                            while 1:
                                tmpdata = portHandler.ser.read()
                                if tmpdata != b'':
                                    responsedatabytes = tmpdata
                                    break
                            for i in range(datalen):
                                responsedatabytes = responsedatabytes + portHandler.ser.read()
                            responsedata = list(responsedatabytes)
                            response.feedback.id2 = responsedata
                            #self.get_logger().info(f"send data : {response.feedback.id2}")
                            break

        #-----------------------------send command motor 3------------------------  
                data = bytes(request.sercommand.id3)
                if data != b'':
                    portHandler.ser.write(data)
                    # print a pretty message
                    self.get_logger().info(f'send m3 : {data}')
                    portHandler.ser.write(call_feedback(3, 0xA1))
                    while(1):
                        #self.get_logger().info('waiting to read m3')
                        if portHandler.ser.readable():
                            #responsedatabytes = self.ser.readline()
                            responsedatabytes = b''
                            while 1:
                                tmpdata = portHandler.ser.read()
                                if tmpdata != b'':
                                    responsedatabytes = tmpdata
                                    break
                            for i in range(datalen):
                                responsedatabytes = responsedatabytes + portHandler.ser.read()
                            responsedata = list(responsedatabytes)
                            response.feedback.id3 = responsedata
                            #self.get_logger().info(f"send data : {response.feedback.id3}")
                            break


                # response state
                response.success = True
            except Exception as e:
                self.get_logger().error(f'move failed!! error : {e}')
                response.success = False

#----------------------------- init ------------------------------  
        elif request.codecommand == "Init":
            self.get_logger().info('Initializing!!')
            self.nuri_init()
            # response state
            response.success = True

#----------------------------- debug data ------------------------  
        elif request.codecommand == "Read":
            self.get_logger().info('waiting to read!!')
            while(1):
                if portHandler.ser.readable():
                    response.feedback = list(portHandler.ser.readline())
                    break


#----------------------------- motor config ------------------------ 

        elif request.codecommand == "Config":
            self.get_logger().info('setup motors!!')

            try:

        #-----------------------------send command motor 0------------------------        
                data = bytes(request.sercommand.id0)
                if data != b'':
                    portHandler.ser.write(data)
                    # print a pretty message
                    self.get_logger().info(f'send m0 : {data}')
                    portHandler.ser.write(call_feedback(0, 0xA1))
                    
                    response.feedback.id0 = []
                           
        #-----------------------------send command motor 1------------------------  
                data = bytes(request.sercommand.id1)
                if data != b'':
                    portHandler.ser.write(data)
                    # print a pretty message
                    self.get_logger().info(f'send m1 : {data}')
                    portHandler.ser.write(call_feedback(1, 0xA1))
                    response.feedback.id1 = []
                            

        #-----------------------------send command motor 2------------------------  
                data = bytes(request.sercommand.id2)
                if data != b'':
                    portHandler.ser.write(data)
                    # print a pretty message
                    self.get_logger().info(f'send m2 : {data}')
                    portHandler.ser.write(call_feedback(2, 0xA1))
                    response.feedback.id2 = []


        #-----------------------------send command motor 3------------------------  
                data = bytes(request.sercommand.id3)
                if data != b'':
                    portHandler.ser.write(data)
                    # print a pretty message
                    self.get_logger().info(f'send m3 : {data}')
                    portHandler.ser.write(call_feedback(3, 0xA1))
                    response.feedback.id3 = []

                # response state
                response.success = True
            except Exception as e:
                self.get_logger().error(f'move failed!! error : {e}')
                response.success = False
            


        else:
            # response state
            response.success = False
        
        # return the response parameter
        self.get_logger().info(f'send response : {response.success}   feedback : {response.feedback}')
        self.get_logger().info("---------------------------------------------")
        return response
    
    def nuri_init(self):
        for i in range(4):
            self.get_logger().info(f'{i} motor initializing...')
            portHandler.ser.write(call_feedback(i, 0xA0))
            count = 0
            while 1:
                if portHandler.ser.readable():
                    readdata = portHandler.ser.readline()
                    id = self.checkID(readdata)
                    if id == i:
                        self.get_logger().info(f'{i} motor initialized!')
                        break
                    elif id == -1:
                        if count == 10:
                            self.get_logger().error(f'{i} motor failed!!')
                            break
                        else:
                            self.get_logger().warn('retry...')
                            portHandler.ser.write(call_feedback(i, 0xA0))
                            count = count + 1
                    else:
                        self.get_logger().error(f'responsed but not {i} motor. retry...')
                        portHandler.ser.write(call_feedback(i, 0xA0))
                        count = count + 1
    def checkID(self, data):
        readdataHEX = data.hex()
        try :
            id = int(readdataHEX[4:6])
            return id
        except:
            self.get_logger().warn('empty data receved.')
            return -1

    #4~6 joint callback
    def callback(self, msg):

        # Write Goal Position

        goal_position1 = int((-msg.data[0] + OFFSET_4) / 0.088)
        goal_position2 = int((msg.data[1] + OFFSET_5) / 0.088)
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
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(f'Failed to set goal position ID{dxl_id}: {packetHandler.getTxRxResult(dxl_comm_result)}')
            else:
                #self.get_logger().info(f'ID{dxl_id} goal position: {round(goal_position)}')
                pass



    def grip_callback(self, request, response):
        grip = request.data
        angle_step = 33
        initial_position = int(360 / 0.088)
        final_position = int(150/ 0.088)
       
        # self.get_logger().info(f'Received grip command: {grip}')
        
        

        if grip: # Gripper 동작 명령받은 경우

#           while(1)
            self.gripper(initial_position)
            for goal_position in range(int(initial_position), int(final_position), -angle_step):
                self.gripper(goal_position)
                self.get_logger().info('-------------------------Gripping-------------------------')
                self.torque() # Current읽는 함수 불러오기
                time.sleep(0.01)

            self.get_logger().info('-------------------------Fnish GRIP-------------------------')


            if self.current <= -120:  #전류값 임계치 이상인지 확인
                response.success = True
                response.message = 'Grip success'
                self.get_logger().info(f'Present current: {self.current}')


            else: #전류값 임계치 이하인지 확인
                response.success = False
                response.message = 'Grip failed'
                self.get_logger().info(f'Present current: {self.current}')
                self.gripper(initial_position)


        else: # Gripper 동작 명령 안 받은 경우
            self.gripper(initial_position)
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
            self.current_init = (dxl_present_current & 0XFFFF)

            if self.current_init >= 65536/2:
               self.current = (65536 - self.current_init)*(-1)
               self.get_logger().error(f'Present current: {self.current}')

            else:
                self.current = self.current_init
                self.get_logger().error(f'Present current: {self.current}')


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
