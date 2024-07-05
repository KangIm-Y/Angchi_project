import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# GStreamer 초기화
Gst.init(None)

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        # 디버그 레벨 설정
        Gst.debug_set_default_threshold(3)

        self.pipeline = Gst.parse_launch(
            "udpsrc port=5000 caps=\"application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96\" ! queue ! rtph264depay ! queue ! avdec_h264 ! queue ! videoconvert ! queue ! appsink name=sink sync=false"
        )
        self.sink = self.pipeline.get_by_name('sink')
        self.sink.connect('new-sample', self.on_new_sample)
        self.pipeline.set_state(Gst.State.PLAYING)

    def on_new_sample(self, sink):
        sample = sink.emit('pull-sample')
        buf = sample.get_buffer()
        caps = sample.get_caps()
        success, map_info = buf.map(Gst.MapFlags.READ)
        self.get_logger().info(f'caps : {caps}   success : {success}   map_info : {map_info}')
        if success:
            self.get_logger().info('asd')
            data = np.frombuffer(map_info.data, dtype=np.uint8) 
            frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
            if frame is not None:
                cv2.imshow('Received Frame', frame)
                cv2.waitKey(1)
            buf.unmap(map_info)
        return Gst.FlowReturn.OK

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.pipeline.set_state(Gst.State.NULL)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
