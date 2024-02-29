import cv2
import mediapipe as mp
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String


class HandDetectNode(Node):
    def __init__(self):
        super().__init__('hand_motion_detector')
        self.cap = cv2.VideoCapture(0)

        self.mpHands = mp.solutions.hands
        self.my_hands = self.mpHands.Hands()
        self.mpDraw = mp.solutions.drawing_utils
        self.compareIndex = [[5, 4], [6, 8], [10, 12], [14, 16], [18, 20]]
        self.gesture = [[True, True, True, True, True, "Hello"],
                [False, False, False, False, False, "nope"],
                [False, True, True, False, False, "yeah"]]

        self.qos_profile = QoSProfile(depth=10)
        self.publisher = self.create_publisher(String, 'arduino_command', self.qos_profile)
        self.timer = self.create_timer(0.1, self.detect_handlandmarks)

    def dist(self, x1, y1, x2, y2):
        return math.sqrt(math.pow(x1 - x2, 2)) + math.sqrt(math.pow(y1 - y2, 2))

    def detect_handlandmarks(self):
        msg = String()

        ret, img = self.cap.read()

        if not ret:
            print("fail to connect")
        else:
            before = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            after = self.my_hands.process(before)

            if after.multi_hand_landmarks:
                for hand_landmarks in after.multi_hand_landmarks:
                    hands_open = [False] * 5  
                    for i in range(0, 5):
                        hands_open[i] = (self.dist(hand_landmarks.landmark[0].x, hand_landmarks.landmark[0].y,
                                              hand_landmarks.landmark[self.compareIndex[i][0]].x,
                                              hand_landmarks.landmark[self.compareIndex[i][0]].y) <
                                         self.dist(hand_landmarks.landmark[0].x, hand_landmarks.landmark[0].y,
                                              hand_landmarks.landmark[self.compareIndex[i][1]].x,
                                              hand_landmarks.landmark[self.compareIndex[i][1]].y))

                    for g in self.gesture:
                        if all(a == b for a, b in zip(g[:5], hands_open)): #zip and then, jogunmun a==b -> all -> one boolean
                            msg.data=str(g[5])
                            break 
                        else :
                            msg.data = str("none")
            
        if(msg.data == '') :
            msg.data=str('none')
        else :
            pass
        self.publisher.publish(msg)
        self.get_logger().info(str(msg.data))
            


    def destroy(self):
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = HandDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
