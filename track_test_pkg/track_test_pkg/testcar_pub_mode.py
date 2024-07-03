import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Int32MultiArray

class Testcar_pub(Node):
    def __init__(self):
        super().__init__('test_car_pub') 
        qos_profile = QoSProfile(depth=10)
        self.publisher = self.create_publisher(Int32MultiArray, 'Odrive_control', qos_profile)
        self.control_sub = self.create_subscription(
            Int32MultiArray,
            'img_joy_data',
            self.circulate_joys,
            qos_profile)
        self.emergency_joy_sub = self.create_subscription(
            Int32MultiArray,
            'joy_data',
            self.emergency_joy,
            qos_profile)

    def publish_msg(self, mode, vel_l, vel_r):
        msg = Int32MultiArray()
        msg.data = [mode, vel_l, vel_r]
        self.publisher.publish(msg)
        self.get_logger().info(f'Pub msg: {msg.data}')  
        
    def circulate_joys(self, msg) :
        ##image processing data sub & pub
        ##### parameters #####
        max_vel = 10
        
        L_end, midpoint, R_end = msg.data
        
        ######################
        
        L_diff = midpoint- L_end
        R_diff = R_end - midpoint
        
        if L_diff > R_diff :
            L_diff 
        
        
        return 0
    
    def emergency_joy(self) :
        ##joy data input & pub
        return 0

def main(args=None):
    rclpy.init(args=args)
    pub = Testcar_pub()

    try:
        while rclpy.ok():
            try:
                ##fix input....
                mode, vel_l, vel_r = map(int, input('CONTROL GOGO: ').split())
                pub.publish_msg(mode, vel_l, vel_r)
            except ValueError: 
                print("FAIL")
    except KeyboardInterrupt:
        pub.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        pub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()