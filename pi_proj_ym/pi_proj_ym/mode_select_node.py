import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Int32

class mode_select(Node):

    def __init__(self):
        super().__init__('mode_select_node')
        qos_profile = QoSProfile(depth=10)
        self.GPIO_set_publisher = self.create_publisher(Int32, 'GPIO_set', qos_profile)
        self.Arduino_set_publisher = self.create_publisher(Int32, 'Arduino_set', qos_profile)
        self.timer = self.create_timer(1, self.publish_msg)
        self.count = 0

    def publish_msg(self):
        user_input = input("  1) GPIO 2) Arduino \n  모드를 선택하세요. : ")
        try:
            input_data = int(user_input)
            rp_out = Int32()
            ad_out = Int32()
            if input_data == 1:
                rp_out.data = 1
                ad_out.data = 0
                self.GPIO_set_publisher.publish(rp_out)
                self.Arduino_set_publisher.publish(ad_out)
                print("\n%d번 GPIO모드를 실행합니다.\n" %input_data)

            elif input_data == 2:
                rp_out.data = 0
                ad_out.data = 1
                self.GPIO_set_publisher.publish(rp_out)
                self.Arduino_set_publisher.publish(ad_out)
                print("\n%d번 Arduino모드를 실행합니다.\n" %input_data)


            else:
                print("잘못된 입력입니다. 다시 입력해주세요. \n")
                return
        except ValueError:
            print("잘못된 입력입니다. 다시 입력해주세요. \n")
            return


def main(args=None):
    rclpy.init(args=args)
    node = mode_select()
    try:
        while rclpy.ok():
            node.publish_msg()

    except KeyboardInterrupt:
        node.get_logger().info('Node Exit :: Keyboard Interrupt')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()