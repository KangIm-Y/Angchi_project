# import the PositionService module from custom_interfaces package
from custom_interfaces.srv import PositionService
from std_srvs.srv import SetBool
# import the ROS2 Python client libraries
import rclpy
from rclpy.node import Node
import sys



class Service(Node):

    def __init__(self):
        # Here you have the class constructor

        # call the class constructor to initialize the node as service_moving
        super().__init__('service_moving')
        # create the Service Server object
        # defines the type, name, and callback function
        self.srv = self.create_service(SetBool, 'ActiveGripper', self.grip_callback)
    
        

    def grip_callback(self, request, response):
        # The callback function receives the self-class parameter, 
        # received along with two parameters called request and response
        # - receive the data by request
        # - return a result as a response
        # print a pretty message
        grip = request.data       # grip이 True면 그리퍼로 물체 잡기 False면 물체 놓기
        self.get_logger().info( f'received grip command : {grip}')
        
        # 여기다가 그리퍼 잡는 함수 넣기 성공하면 response.success에다가 True, 실패하면 False 넣기
        response.success = True
        # 성공/실패시 같이 보낼 메시지는 response.message에다가 string 형식으로 넣을 것.
        response.message = 'grip success!!!'

        
        # return the response parameter 이거 리턴하는 순간 클라이언트한테 응답 보내는거임
        return response


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    moving_service = Service()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(moving_service)
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()