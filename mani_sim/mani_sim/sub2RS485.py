import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
import serial

global header1
global header2
global id
global dataSize
global sumpose

#poseArr = []


header1 = 0xFF
header2 = 0xFE
id = 0x00
dataSize = 0x07
sumpose = 0x00
PI = 3.141592653589793


class JointStateSubscriber(Node):

    def __init__(self):
        super().__init__('JointState_subscriber')
        qos_profile = QoSProfile(depth=10)
        self.jointstate_subscriber = self.create_subscription(
            JointState,
            'joint_states',
            self.subscribe_topic_message,
            qos_profile)

        self.timer = self.create_timer(0.5, self.send_serial_data)

        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=3)
        self.trig = 0
        self.chgTrig = 0
        self.poseArrRad = []
        self.poseArr = [0,0,0,0,0,0]
        self.poseArrOld = [90,130,50,90,90,90]
        self.poseArrHex = [0x00,0x00,0x00,0x00,0x00,0x00]

    def subscribe_topic_message(self, msg):
        self.poseArrRad = msg.position
        for i in range(0,len(self.poseArrRad)): #change rad 2 deg
            # print("joint rad",i,"::",self.poseArrRad[i])
            self.poseArrRad[i] = int(round((180 / PI) * self.poseArrRad[i]))
            if(i == 0):
                self.poseArr[0] = 90 - self.poseArrRad[i] 
            elif(i == 1):
                self.poseArr[2] = - self.poseArrRad[i]
            elif(i == 2):
                self.poseArr[3] = self.poseArrRad[i] + 90
            elif(i == 3):
                self.poseArr[1] = 180- self.poseArrRad[i]
            elif(i == 4):
                self.poseArr[4] = self.poseArrRad[i] + 90
            elif(i == 5):
                self.poseArr[5] = 90
        print(" ")
        

        for i in range(0,len(self.poseArr)): #print to check rad2deg
            print("joint deg",i,"::",self.poseArr[i])
        print(" ")

        for i in range(0,len(self.poseArr)): #print to check rad2deg
            print("joint old",i,"::",self.poseArrOld[i])
        print(" ")

        self.trig = 1
        # for i in range(0,len(self.poseArr)): #check position data is changed
        #     if(self.poseArr[i] != self.poseArrOld[i]):
        #         self.chgTrig += 1
        # if(self.chgTrig > 0): # when position data is changed, then RS485 send trig is true
        #     self.trig = 1
        #     # print("chgtrig",self.chgTrig)
        #     self.chgTrig = 0
        # self.poseArrOld = self.poseArr
        # print("trig",self.trig)


    def send_serial_data(self):
        sumpose = 0x00
        if self.trig == 1:
            for i in range(0,len(self.poseArr)):
                # print(self.poseArr[i],type(self.poseArr[i]))
                self.poseArrHex[i] = int(hex(int(self.poseArr[i])),16)
                print("joint Hex",i,bytes([self.poseArrHex[i]]))
                sumpose += self.poseArrHex[i]
            
            checkSum = (~ (id + dataSize + sumpose)) & 0xFF
            # print("BBBB")
            print("header1",type(bytes([header1])))
            print(bytes([header1]))
            print("arrhex",type(bytes([self.poseArrHex[0]])))
            print(bytes([self.poseArrHex[0]]))
            rs485_send = bytes([header1])+bytes([header2])+bytes([id])\
                +bytes([dataSize])+bytes([checkSum])\
                    +bytes([self.poseArrHex[0]])+bytes([self.poseArrHex[1]])+bytes([self.poseArrHex[2]])\
                    +bytes([self.poseArrHex[3]])+bytes([self.poseArrHex[4]])+bytes([self.poseArrHex[5]])
            print('send data=' + rs485_send.hex())
            self.ser.write(rs485_send)
            self.trig = 0
        else:
            return

def main(args=None):
    rclpy.init(args=args)
    node = JointStateSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
