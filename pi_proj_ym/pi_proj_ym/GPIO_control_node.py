import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Int32
import RPi.GPIO as GPIO

led_pin = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(led_pin, GPIO.OUT)

# PWM 설정
pwm_frequency = 1000  # PWM 주파수 (Hz)
pwm_duty_cycle = 0   # 초기 듀티 사이클 (0%로 시작)

# PWM 객체 생성
pwm_led = GPIO.PWM(led_pin, pwm_frequency)
pwm_led.start(pwm_duty_cycle)

class GPIO_control(Node):

    def __init__(self):
        super().__init__('GPIO_control_node')
        qos_profile = QoSProfile(depth=10)
        self.GPIO_control_node = self.create_subscription(
            Int32,
            'GPIO_set',
            self.subscribe_topic_message,
            qos_profile)     
        self.last_msg_data = 0
        self.timer = self.create_timer(1.0, self.LED_control)

    def subscribe_topic_message(self, msg):        
        self.last_msg_data = msg.data
        print("Terminal Data Input: ", self.last_msg_data)

    def LED_control(self):
        input_data = self.last_msg_data

        if input_data == 1:
            print("LED_control HIGH")
            pwm_led.ChangeDutyCycle(100)  # 듀티 사이클을 100%로 설정하여 LED를 켬
            
        elif input_data == 0:
            print("LED_control LOW")
            pwm_led.ChangeDutyCycle(0)    # 듀티 사이클을 0%로 설정하여 LED를 끔
            
        else:
            print("Data Error \n")
            return

def main(args=None):
    rclpy.init(args=args)
    node = GPIO_control()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pwm_led.stop()  # 프로그램 종료 시 PWM 정지
        GPIO.cleanup()
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
