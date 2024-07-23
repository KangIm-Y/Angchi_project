# import the Twist module from geometry_msgs messages interface
from geometry_msgs.msg import Twist
# import the MyCustomServiceMessage module from custom_interfaces package
from custom_interfaces.srv import Protocool
# import the ROS2 Python client libraries
import time as t
from struct import pack
import rclpy
from rclpy.node import Node
import serial

class Service(Node):

    def __init__(self):
        # Here you have the class constructor
        # call the class constructor to initialize the node as service_stop
        super().__init__('serial_comm')
        # create the Service Server object
        # defines the type, name, and callback function
        self.srv = self.create_service(Protocool, 'command', self.custom_service_callback)
        self.ser = serial.Serial('/dev/ttyRS485', 9600, timeout=0.1)
        

    def custom_service_callback(self, request, response):
        self.get_logger().info(f'received : {request.sercommand} , {request.codecommand}')
        ser_comm_byte = bytes(request.sercommand)

        if ser_comm_byte != b'':
            self.ser.write(ser_comm_byte)
            # print a pretty message
            self.get_logger().info(f'send : {ser_comm_byte}')
            # response state
            response.success = True
        elif request.codecomand == "Init":
            self.get_logger().info('Initializing!!')
            self.nuri_init()
            # response state
            response.success = True
        else:
            # response state
            response.success = False
        
        # return the response parameter
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