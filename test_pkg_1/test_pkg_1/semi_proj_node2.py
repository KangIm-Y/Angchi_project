import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
import time


class SemiProjectNode1(Node):

    def __init__(self):
        super().__init__('node2')
        qos_profile = QoSProfile(depth=10)
        
        self.count = 0
        self.start_flag = False #pub start flag

        self.subscriber1 = self.create_subscription(
            String,
            'order',
            self.order_callback,
            qos_profile
        )
        
        self.subscriber2 = self.create_subscription(
            String,
            'node1_pub',
            self.node1_pub_callback,
            qos_profile
        )
        
        self.publisher1 = self.create_publisher(
            String,
            'node2_pub',
            qos_profile
        )
        
        self.timer = self.create_timer(1, self.publisher1_msg)

    def order_callback(self, msg):
        if msg.data == "stop" :
            self.start_flag = False
            self.get_logger().info('stopped by master')
            return
        elif msg.data== '2' :
            self.start_flag = True
            self.get_logger().info(f'node2 start first. at count 0')
            self.publisher1_msg()
        elif msg.data == '1' :
            self.start_flag = True
        else :
            self.get_logger().info('worng data')
            return
        
    def node1_pub_callback(self,msg) :
        if self.start_flag == True :
            self.get_logger().info(f'Ive got msg.data : {msg.data} from node1')
            self.count += 1
        else : 
            return
        
    def publisher1_msg(self) :
        if self.start_flag == True :
            msg = String()  
            msg.data = f'publish data : {self.count}'
            self.publisher1.publish(msg)
        else :
            return
        


def main(args=None):
    rclpy.init(args=args)
    node = SemiProjectNode1()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()