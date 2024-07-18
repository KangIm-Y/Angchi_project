import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import Int32MultiArray

import serial
import time

# 시리얼 포트 설정
ser1 = serial.Serial(port='/dev/ttyRS232_0',\
                    baudrate=9600,\
                    parity=serial.PARITY_NONE,\
                    stopbits=serial.STOPBITS_ONE,\
                    bytesize=serial.EIGHTBITS,\
                    timeout=0)
ser2 = serial.Serial(port='/dev/ttyRS232_1',\
                    baudrate=9600,\
                    parity=serial.PARITY_NONE,\
                    stopbits=serial.STOPBITS_ONE,\
                    bytesize=serial.EIGHTBITS,\
                    timeout=0)


class detectdis(Node):
    def __init__(self):
        super().__init__('detectdis')
        qos_profile = QoSProfile(depth=10)
        self.detectdis = self.create_publisher(Int32MultiArray, 'distance' , qos_profile)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.count = 0
        serport1 = ser1.portstr
        serport2 = ser2.portstr
        self.get_logger().info(f'Sensor1 found at {serport1}')
        self.get_logger().info(f'Sensor1 found at {serport2}')
        self.sendData(0x32)
        self.sendData(0x32)
        self.sendData(0x32)
        self.get_logger().info(f'Initialize sensor : {self.readData()}')

    def timer_callback(self):
        
        dis = Int32MultiArray()
        dis.data = self.readDis()
        self.detectdis.publish(dis)
        self.get_logger().info(f'[{self.count}] : {dis.data}')
        self.count += 1
    

    def sendData(self, data):
        ser1.write(bytes(bytearray([data])))
        ser2.write(bytes(bytearray([data])))
        print("data send", data)

    def readData(self):
        readdata = [0,0]
        while 1:
            if ser1.readable():
                readbit1 = ser1.readline()
                try:
                    readdata[0] = int(readbit1.decode("utf-8"))
                except:
                    # print("before")
                    # print(type(readdata))
                    # print(readdata)
                    readdata[0] = 999
                    # print(" after")                    
                    # print(type(readdata))
                    # print(readdata)
                
                # print(" final")
                # print(type(readdata))
                # print(readdata)
                break
           # else:
            #    time.sleep(0.1)
        while 1:
            if ser2.readable():
                readbit2 = ser2.readline()
                try:
                    readdata[1] = int(readbit2.decode("utf-8"))
                except:
                    # print("before")
                    # print(type(readdata))
                    # print(readdata)
                    readdata[1] = 999
                    # print(" after")
                    # print(type(readdata))
                    # print(readdata)
                
                # print(" final")
                # print(type(readdata))
                # print(readdata)
                break
           # else:
            #    time.sleep(0.1)
        return readdata

    def readDis(self):
        self.sendData(0x00)
        
        disdata = self.readData()
        time.sleep(0.1)
        return disdata



def main(args = None):
    rclpy.init(args=args)
    node = detectdis()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        ser1.close()
        ser2.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
