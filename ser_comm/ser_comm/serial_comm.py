2# import the MyCustomServiceMessage module from custom_interfaces package
from custom_interfaces.srv import Protocool
# import the ROS2 Python client libraries
import time 
from struct import pack
import rclpy
from rclpy.node import Node
import serial


datalen = 12

class Service(Node):

    def __init__(self):
        super().__init__('serial_comm')
        self.srv = self.create_service(Protocool, 'command', self.custom_service_callback)
        self.ser = serial.Serial('/dev/ttyRS485', 57600, timeout=0.01)
        

    def custom_service_callback(self, request, response):
        self.get_logger().info(f'received : {request.sercommand} , {request.codecommand}')

        if request.codecommand == "Move":
            try:

        #-----------------------------send command motor 0------------------------        
                data = bytes(request.sercommand.id0)
                if data != b'':
                    self.ser.write(data)
                    # print a pretty message
                    self.get_logger().info(f'send m0 : {data}')
                    self.ser.write(call_feedback(0, 0xA1))
                    while(1):
                        #self.get_logger().info('waiting to read m0')
                        if self.ser.readable():
                            responsedatabytes = b''
                            while 1:
                                tmpdata = self.ser.read()
                                if tmpdata != b'':
                                    responsedatabytes = tmpdata
                                    break
                            for i in range(datalen):
                                responsedatabytes = responsedatabytes + self.ser.read()
                            responsedata = list(responsedatabytes)
                            response.feedback.id0 = responsedata
                            #self.get_logger().info(f"send data : {response.feedback.id0}")
                            break

        #-----------------------------send command motor 1------------------------  
                data = bytes(request.sercommand.id1)
                if data != b'':
                    self.ser.write(data)
                    # print a pretty message
                    self.get_logger().info(f'send m1 : {data}')
                    self.ser.write(call_feedback(1, 0xA1))
                    while(1):
                        #self.get_logger().info('waiting to read m1')
                        if self.ser.readable():
                            #responsedatabytes = self.ser.readline()
                            responsedatabytes = b''
                            while 1:
                                tmpdata = self.ser.read()
                                if tmpdata != b'':
                                    responsedatabytes = tmpdata
                                    break
                            for i in range(datalen):
                                responsedatabytes = responsedatabytes + self.ser.read()
                            #self.get_logger().info(f"feedback data : {responsedatabytes}")
                            responsedata = list(responsedatabytes)
                            response.feedback.id1 = responsedata
                            #self.get_logger().info(f"send data : {response.feedback.id1}")
                            break

        #-----------------------------send command motor 2------------------------  
                data = bytes(request.sercommand.id2)
                if data != b'':
                    self.ser.write(data)
                    # print a pretty message
                    self.get_logger().info(f'send m2 : {data}')
                    self.ser.write(call_feedback(2, 0xA1))
                    while(1):
                        #self.get_logger().info('waiting to read m2')
                        if self.ser.readable():
                            #responsedatabytes = self.ser.readline()
                            responsedatabytes = b''
                            while 1:
                                tmpdata = self.ser.read()
                                if tmpdata != b'':
                                    responsedatabytes = tmpdata
                                    break
                            for i in range(datalen):
                                responsedatabytes = responsedatabytes + self.ser.read()
                            responsedata = list(responsedatabytes)
                            response.feedback.id2 = responsedata
                            #self.get_logger().info(f"send data : {response.feedback.id2}")
                            break

        #-----------------------------send command motor 3------------------------  
                data = bytes(request.sercommand.id3)
                if data != b'':
                    self.ser.write(data)
                    # print a pretty message
                    self.get_logger().info(f'send m3 : {data}')
                    self.ser.write(call_feedback(3, 0xA1))
                    while(1):
                        #self.get_logger().info('waiting to read m3')
                        if self.ser.readable():
                            #responsedatabytes = self.ser.readline()
                            responsedatabytes = b''
                            while 1:
                                tmpdata = self.ser.read()
                                if tmpdata != b'':
                                    responsedatabytes = tmpdata
                                    break
                            for i in range(datalen):
                                responsedatabytes = responsedatabytes + self.ser.read()
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
                if self.ser.readable():
                    response.feedback = list(self.ser.readline())
                    break

#----------------------------- motor config ------------------------ 

        elif request.codecommand == "Config":
            self.get_logger().info('setup motors!!')

            try:

        #-----------------------------send command motor 0------------------------        
                data = bytes(request.sercommand.id0)
                if data != b'':
                    self.ser.write(data)
                    # print a pretty message
                    self.get_logger().info(f'send m0 : {data}')
                    self.ser.write(call_feedback(0, 0xA1))
                    
                    response.feedback.id0 = []
                           
        #-----------------------------send command motor 1------------------------  
                data = bytes(request.sercommand.id1)
                if data != b'':
                    self.ser.write(data)
                    # print a pretty message
                    self.get_logger().info(f'send m1 : {data}')
                    self.ser.write(call_feedback(1, 0xA1))
                    response.feedback.id1 = []
                            

        #-----------------------------send command motor 2------------------------  
                data = bytes(request.sercommand.id2)
                if data != b'':
                    self.ser.write(data)
                    # print a pretty message
                    self.get_logger().info(f'send m2 : {data}')
                    self.ser.write(call_feedback(2, 0xA1))
                    response.feedback.id2 = []


        #-----------------------------send command motor 3------------------------  
                data = bytes(request.sercommand.id3)
                if data != b'':
                    self.ser.write(data)
                    # print a pretty message
                    self.get_logger().info(f'send m3 : {data}')
                    self.ser.write(call_feedback(3, 0xA1))
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
            self.ser.write(call_feedback(i, 0xA0))
            count = 0
            while 1:
                if self.ser.readable():
                    readdata = self.ser.readline()
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
                            self.ser.write(call_feedback(i, 0xA0))
                            count = count + 1
                    else:
                        self.get_logger().error(f'responsed but not {i} motor. retry...')
                        self.ser.write(call_feedback(i, 0xA0))
                        count = count + 1
    def checkID(self, data):
        readdataHEX = data.hex()
        try :
            id = int(readdataHEX[4:6])
            return id
        except:
            self.get_logger().warn('empty data receved.')
            return -1



def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor  
    service = Service()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    try:
        rclpy.spin(service)
    except KeyboardInterrupt:
        service.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()






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
