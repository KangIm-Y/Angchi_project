import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState

PI = 3.141592653589793


class Tf_jointState(Node):

    def __init__(self):
        super().__init__('Tf_jointState')
        qos_profile = QoSProfile(depth=10)
        self.jointstate_subscriber = self.create_subscription(
            JointState,
            'joint_states',
            self.subscribe_topic_message,
            qos_profile)
        
        self.publisher_ = self.create_publisher(Int32MultiArray, 'joint', qos_profile)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.trig = 0
        self.chgTrig = 0
        self.poseArrRad = []
        self.poseArr = [0,0,0,0,0,0]


    def subscribe_topic_message(self, msg):
        self.poseArrRad = msg.position
        #self.get_logger().info(f'poseArr : {self.poseArrRad}')
        for i in range(0,len(self.poseArrRad)): #change rad 2 deg
            if i == 0 :
                self.poseArr[1] = int(round((180 / PI) * self.poseArrRad[i]))
                self.poseArr[1] = int(round(self.poseArr[1] * 4.8))
            elif i == 1 :
                self.poseArr[2] = int(round((180 / PI) * self.poseArrRad[i]))
            elif i == 2 :
                self.poseArr[3] = int(round((180 / PI) * self.poseArrRad[i]))
            elif i == 3 :
                self.poseArr[4] = int(round((180 / PI) * self.poseArrRad[i]))
            elif i == 4 :
                self.poseArr[0] = int(round((180 / PI) * self.poseArrRad[i]))
            elif i == 5 :
                self.poseArr[5] = int(round((180 / PI) * self.poseArrRad[i]))
            elif i == 6:
                pass
            else:
                self.get_logger().error("joint states list is too long!!")

        print(" ")
        
        for i in range(0,len(self.poseArr)): #print to check rad2deg
            print("joint deg",i,"::",self.poseArr[i])
        print(" ")

    def timer_callback(self):
        msg = Int32MultiArray()
        msg.data = self.poseArr[0:4]
        # Publish the message to the Topic
        self.publisher_.publish(msg)
        # Display the message on the console
        self.get_logger().info('Publishing: "%s"' % msg)


def main(args=None):
    rclpy.init(args=args)
    node = Tf_jointState()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
