import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Int32MultiArray
import serial
import time as t
from struct import pack
from custom_interfaces.srv import String
#from nuri_protocool import *





class JointSubscriber(Node):

    def __init__(self):
        super().__init__('joint_Subscriber')
        qos_profile = QoSProfile(depth=10)
        self.joint_Subscriber = self.create_subscription(
            Int32MultiArray,
            'joint',
            self.subscribe_topic_message,
            qos_profile)
        self.client = self.create_client(String, 'movement')


        self.file_path = "degarr.txt"
        self.posarray = [0,0,0,0]
        self.ser = serial.Serial('/dev/ttyRS485', 9600, timeout=0.1)
        self.read_pos()
        self.nuri_init()
        
        
        

        


    def subscribe_topic_message(self, msg):
        self.posarray = self.inv_data(msg.data)
        self.get_logger().info('Received message: {0}'.format(self.posarray))
        self.store_pos()
        self.pos_nuri()
    
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

                elif count == 3:
                    self.get_logger().error(f'{i} motor failed!!')
                    break
                else:
                    self.get_logger().warn(f'{i} motor no response. retry...')
                    count = count + 1
        self.set_nuri()
        t.sleep(2)
        self.nuri_initpos()

    def nuri_initpos(self):
        self.get_logger().info('Moving to zeropos.')
        pos_inv = []
        for i in self.posarray:
            pos_inv.append(~i + 1)
        self.posarray = pos_inv
        self.pos_nuri()
        t.sleep(1)
        self.set_nuri_zero()


                    

    def checkID(self, data):
        readdataHEX = data.hex()
        try :
            id = int(readdataHEX[4:6])
            return id
        except:
            self.get_logger().warn('empty data receved.')
            return -1

    
    def set_nuri(self):
        for i in range(4):
            self.ser.write(set_pos_con_mode(i, 0))


    def set_nuri_zero(self):
        for i in range(4):
            self.ser.write(init_pos(i))
        

    
    def pos_nuri(self):
        id = 0
        for i in range(4):
            self.ser.write(set_degrpm(id, self.posarray[i]))
            self.get_logger().info(f'motor : {id} / deg : {self.posarray[i]} ')
            id = id + 1
        self.get_logger().info('------------------------------------')

    def read_pos(self):
        deg = [0,0,0,0]
        try:
            with open(self.file_path, 'r') as f:
                lines = f.readlines()
                if len(lines) != 4 :
                    self.get_logger().warn("Last data is not exist. All posdata will be zero.")
                else:
                    num = 0
                    for line in lines:
                        try:
                            read_deg = int(line.strip())
                            deg[num] = read_deg
                            self.get_logger().info(f"last {num} joint data is {read_deg}.")
                        except:
                            self.get_logger().warn(f"Can't read data for {num} joint. It will be zero.")
                        num += 1
        except:
            self.get_logger().warn("File not exist. All posdata will be zero and generate new file.")
            self.store_pos()
        self.posarray = deg


    def store_pos(self):
        with open(self.file_path, 'w') as f:
            for i in self.posarray:
                f.write(f"{i}\n")
        self.get_logger().info(f"deg array saved. saved data is {self.posarray}")


    def inv_data(self, data):
        inv = []
        for i in range(len(data)):
            inv.append((~(data[i]) + 1))
        return inv
        
        





def main(args=None):
    rclpy.init(args=args)
    node = JointSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    
    
    
    
    
    
#nuri_protocool_by_ep

def set_degrpm(Id, Deg, Rpm = 5): 
    motor_id = format(Id, '#04x')
    data_num = '0x07'
    mode = '0x01'
    if Deg < 0:
        dir = '0x00'
    else:
        dir = '0x01'
    pos = format(abs(Deg * 100), '#06x')
    pos1 = pos[:4]
    pos2 = '0x' + pos[4:]
    if Rpm >= 0:
        rpm = Rpm * 10
        motor_rpm = format(rpm, '#06x')
        motor_rpm1 = motor_rpm[:4]
        motor_rpm2 = '0x' + motor_rpm[4:]
    data_array = [motor_id, data_num, mode, dir, pos1, pos2, motor_rpm1, motor_rpm2]
    #print(data_array)
    return attach_checksum(data_array)
    
def set_degtime(Id, Deg, Time):
    motor_id = format(Id, '#04x')
    data_num = '0x06'
    mode = '0x02'
    if Deg < 0:
        dir = '0x00'
    else:
        dir = '0x01'
    pos = format(abs(Deg * 100), '#06x')
    pos1 = pos[:4]
    pos2 = '0x' + pos[4:]
    if Time >= 0:
        time = format(Time, '#06x')
        time1 = time[:4]
        time2 = '0x' + time[4:]
    data_array = [motor_id, data_num, mode, dir, pos1, pos2, time1, time2]
    return attach_checksum(data_array)

def set_id(old_id, new_id):
    motor_old_id = format(old_id, '#04x')
    data_num = '0x03'
    mode = '0x06'
    motor_new_id = format(new_id, '#04x')
    data_array = [motor_old_id, data_num, mode, motor_new_id]
    return attach_checksum(data_array)
    
def set_pos_con_mode(Id, conmode):
    motor_id = format(Id, '#04x')
    data_num = '0x03'
    mode = '0x0B'
    Conmode = format(conmode, '#04x')
    data_array = [motor_id, data_num, mode, Conmode]
    return attach_checksum(data_array)

def init_pos(Id):
    motor_id = format(Id, '#04x')
    data_num = '0x02'
    mode = '0x0C'
    data_array = [motor_id, data_num, mode]
    return attach_checksum(data_array)

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