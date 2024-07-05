import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

# GStreamer 초기화
Gst.init(None)

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        # 디버그 레벨 설정
        Gst.debug_set_default_threshold(3)

        self.pipeline = Gst.parse_launch(
            "appsrc name=source caps=video/x-raw,format=BGR,width=1280,height=720,framerate=10/1 ! queue ! videoconvert ! queue ! x264enc tune=zerolatency ! queue ! rtph264pay config-interval=1 pt=96 ! queue ! udpsink host=127.0.0.1 port=5000"
        )
        self.source = self.pipeline.get_by_name('source')
        self.source.set_property('format', Gst.Format.TIME)
        self.source.set_property('is-live', True)
        self.source.set_property('do-timestamp', True)
        self.pipeline.set_state(Gst.State.PLAYING)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('initialized')

    def timer_callback(self):
        # Create a dummy image for demonstration
        frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        frame = cv2.putText(frame, 'ROS2 RTP Stream', (100, 360), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3, cv2.LINE_AA)
        
        data = frame.tobytes()
        buf = Gst.Buffer.new_allocate(None, len(data), None)
        buf.fill(0, data)
        self.source.emit('push-buffer', buf)
        self.get_logger().info(type(data))

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.pipeline.set_state(Gst.State.NULL)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
