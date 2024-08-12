import rclpy
from rclpy.node import Node
from custom_interfaces.srv import PositionService
import time

class PositionServiceServer(Node):
    def __init__(self):
        super().__init__('position_service_server')
        self.srv = self.create_service(PositionService, 'object_coordinate', self.handle_position_service)

    def handle_position_service(self, request, response):
        self.get_logger().info(f'Received coordinates: {request.coordinate}')
        response.success = True
        time.sleep(10)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = PositionServiceServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
