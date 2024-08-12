import rclpy
from rclpy.node import Node
from custom_interfaces.srv import PositionService

class PositionServiceServer(Node):
    def __init__(self):
        super().__init__('position_service_server')
        self.client = self.create_client(PositionService, 'object_coordinate')
        # while not self.client.wait_for_service(timeout_sec=1.0):
        #     # if it is not available, a message is displayed
        #     self.get_logger().info('service not available, waiting again...')
        
        # create an Empty request
        self.mission_timer = self.create_timer(3, self.call_service_periodically)
        self.req = PositionService.Request()
        

    def call_service_periodically(self):
        if self.client.service_is_ready():
            request = PositionService.Request()
            request.coordinate.x = 5.
            request.coordinate.y = 10.
            request.coordinate.z = 15.
            future = self.client.call_async(request)
            future.add_done_callback(self.callback_function)
        else:
            self.get_logger().warn('Service not available')

    def callback_function(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Result: {response.success}')
        except Exception as e:
            self.get_logger().error(f'Service call failed {e}')

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
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
    