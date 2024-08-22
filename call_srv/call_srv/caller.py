# import the PositionService module from custom_interfaces package
from custom_interfaces.srv import PositionService
from std_srvs.srv import SetBool
# import the ROS2 Python client libraries
import rclpy
from rclpy.node import Node
import sys
import time as t


class ClientAsync(Node):

    def __init__(self):
        # Here you have the class constructor

        # call the class constructor to initialize the node as movement_client
        super().__init__('movement_client')
        # create the Service Client object
        # defines the name and type of the Service Server you will work with.
        self.client = self.create_client(PositionService, 'pos_srv')
        self.client1 = self.create_client(SetBool, 'ActiveGripper')
        # checks once per second if a Service matching the type and name of the client is available.
        while not self.client.wait_for_service(timeout_sec=1.0):
            # if it is not available, a message is displayed
            self.get_logger().info('Moveit service not available, waiting again...')
        #while not self.client1.wait_for_service(timeout_sec=1.0):
            # if it is not available, a message is displayed
            #self.get_logger().info('Gripper service not available, waiting again...')
        # create an Empty request
        self.req = PositionService.Request()
        self.grip = SetBool.Request()
        

    def send_request(self):
        arr = []
        for i in range(3):
            arr.append(float(input("[0,0,0] is zeropos, [-1,-1,-1] is homepos, anything else is coordinate. input : ")))


        # send the request
        self.req.coordinate.x = arr[0]
        self.req.coordinate.y = arr[1]
        self.req.coordinate.z = arr[2]

        if arr == [0,0,0]:
            self.req.pose = 'zero'
        elif arr == [-1,-1,-1]:
            self.req.pose = 'home'
        else:
            self.req.pose = ''
        # uses sys.argv to access command line input arguments for the request.
        self.future = self.client.call_async(self.req)
        # to print in the console

    def active_gripper(self, grip):
        # send the request
        self.grip.data = grip
        # uses sys.argv to access command line input arguments for the request.
        self.future1 = self.client1.call_async(self.grip)
        self.get_logger().info('Gripper server called!') 
        # to print in the console

def main(args=None):
    moved = 0
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    client = ClientAsync()
    # run the send_request() method
    client.send_request()

    while rclpy.ok():
        # pause the program execution, waits for a request to kill the node (ctrl+c)
        rclpy.spin_once(client)
        if moved == 0:
            if client.future.done():
                try:
                    # checks the future for a response from the Service
                    # while the system is running. 
                    # if the Service has sent a response, the result will be written
                    # to a log message.
                    response = client.future.result()
                    if response.success == True:
                        moved = 1
                        t.sleep(5)
                        client.active_gripper(True)
                    else:
                        client.get_logger().info(
                        'move failed')
                except Exception as e:
                    # Display the message on the console
                    client.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    # Display the message on the console
                    client.get_logger().info(
                        'Response state pos done :  %r' % (response.success,))
        else:
            if client.future1.done():
                try:
                    # checks the future for a response from the Service
                    # while the system is running. 
                    # if the Service has sent a response, the result will be written
                    # to a log message.
                    response1 = client.future1.result()
                    if response1.success == True:
                        client.get_logger().info(
                        'grip succeed ')
                except Exception as e:
                    # Display the message on the console
                    client.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    # Display the message on the console
                    client.get_logger().info(
                        'Response grip state %r' % (response1.success,))
                    break
            

    client.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()



# x -0.2  y -0.46 z 0.07
