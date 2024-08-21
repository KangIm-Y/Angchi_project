import odrive
#import the Odrivetool libraries
import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist interface from the geometry_msgs package
from odrive_ep_interfaces.msg import Motor
from odrive_ep_interfaces.srv import OdriveMode

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.qos import ReliabilityPolicy, QoSProfile

import time

num_threads = 2
odrive_serial_id = "3369356B3233"                  #바꿔줄것

calibration_time = 15




class Odrive_ep(Node):

    def __init__(self):
        super().__init__('odrive_ep')
        

        self.goal_pos = [0.0, 0.0]
        self.goal_vel = [0.0, 0.0]

        self.odrv = odrive.find_any(serial_number=odrive_serial_id)
        self.get_logger().info(f"odrv connected.  voltage : {self.odrv.vbus_voltage}")
        self.init_odrive()
        
        self.get_logger().info("## DEFAULT Node Callback Group=" +
              str(self.default_callback_group))

        self.get_logger().warning("Setting Reentrant Groups")
        # If you set the group reentrant, any Callback inside will be executed in parallel
        # If there are enough threads
        self.group1 = ReentrantCallbackGroup()
        self.get_logger().warning("ReentrantCallbackGroup Set")

        self.publisher_ = self.create_publisher(Motor, 'current_posvel', 10, callback_group=self.group1)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback, callback_group=self.group1)

        self.subscriber = self.create_subscription(
            Motor,
            'goal_posvel',
            self.listener_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
            callback_group=self.group1)

        self.srv = self.create_service(
            OdriveMode, 'odrive_mode_srv', self.SetMode_callback, callback_group=self.group1)
        
    def timer_callback(self):
        msg = Motor()
        current_posvel = self.read_posvel()

        self.odrv.axis0.watchdog_feed()
        self.odrv.axis1.watchdog_feed()
        # pub current pos
        msg.motor0.pos = current_posvel[0]
        # pub current vel
        msg.motor0.vel = current_posvel[1]

        msg.motor1.pos = current_posvel[2]
        msg.motor1.vel = current_posvel[3]

        # Publish the message to the Topic
        self.publisher_.publish(msg)
        # Display the message on the console
        self.get_logger().info('Publishing: "%s"' % msg)

        self.set_pos(self.goal_pos)
        self.set_vel(self.goal_vel)

    
    def listener_callback(self, msg):
        self.get_logger().info(f"set goal {msg}")
        self.goal_pos = [msg.motor0.pos, msg.motor1.pos]
        self.goal_vel = [msg.motor0.vel, msg.motor1.vel]


    def SetMode_callback(self, request, response):
        mode = request.mode
        self.get_logger().warning(f"Set control mode to {mode}...")
        if mode == "Pos":
            self.set_traj_control()
            self.get_logger().warning(f"Set control mode to pos_traj...DONE")
            response.success = True
        elif mode == "Vel":
            self.set_vel_control()
            self.get_logger().warning(f"Set control mode to vel_ramp...DONE")
            response.success = True
        else:
            self.get_logger().error(f"no mode for {mode}")
            response.success = False

        # return the response parameters
        return response
        

    def init_odrive(self):
        self.get_logger().info("----------------------------Calibration--------------------------------")
        self.get_logger().info("Calibrating motors...")
        self.odrv.axis0.requested_state = 3 # AXIS_STATE_FULL_CALIBRATION
        self.odrv.axis1.requested_state = 3 # AXIS_STATE_FULL_CALIBRATION
        time.sleep(calibration_time)
        if self.odrv.axis0.motor.is_calibrated == 1 and self.odrv.axis1.motor.is_calibrated == 1:
            self.get_logger().info("Motor successfully calibrated! Proceeding...")
        else:
            self.get_logger().fatal("Could not calibrate motor. Something is wrong.")

        if self.odrv.axis0.encoder.is_ready == 1 and self.odrv.axis1.encoder.is_ready == 1:
            self.get_logger().info("Encoder offset successfully calibrated! Proceeding...")
        else:
            self.get_logger().fatal("Could not calibrate encoder offset. Something is wrong.")
        
        
        self.odrv.axis0.requested_state = 8         # CLOSED_LOOP_CONTROL
        self.odrv.axis1.requested_state = 8         # CLOSED_LOOP_CONTROL

        self.odrv.axis0.config.enable_watchdog = True
        self.odrv.axis1.config.enable_watchdog = True

        self.odrv.axis0.config.watchdog_timeout = 1
        self.odrv.axis1.config.watchdog_timeout = 1

        self.odrv.axis0.trap_traj.config.vel_limit = 40
        self.odrv.axis0.trap_traj.config.accel_limit = 20
        self.odrv.axis0.trap_traj.config.decel_limit = 20
        self.odrv.axis1.trap_traj.config.vel_limit = 40
        self.odrv.axis1.trap_traj.config.accel_limit = 20
        self.odrv.axis1.trap_traj.config.decel_limit = 20


    
    def set_traj_control(self):

        self.odrv.axis0.controller.input_pos = self.odrv.axis0.encoder.pos_estimate
        self.odrv.axis1.controller.input_pos = self.odrv.axis1.encoder.pos_estimate

        self.odrv.axis0.controller.config.input_mode = 5         #TRAP_TRAJ
        self.odrv.axis1.controller.config.input_mode = 5         #TRAP_TRAJ

    def set_vel_control(self):
        self.odrv.axis0.controller.config.vel_ramp_rate = 20
        self.odrv.axis1.controller.config.vel_ramp_rate = 20
        self.odrv.axis0.controller.config.input_mode = 2         #VEL_RAMP
        self.odrv.axis1.controller.config.input_mode = 2         #VEL_RAMP

    def set_pos(self, pos):
        self.odrv.axis0.controller.input_pos = pos[0]
        self.odrv.axis1.controller.input_pos = pos[1]
    def set_vel(self, vel):
        self.odrv.axis0.controller.input_vel = vel[0]
        self.odrv.axis1.controller.input_vel = vel[1]
    def read_posvel(self):
        axis0_pos = self.odrv.axis0.encoder.pos_estimate
        axis1_pos = self.odrv.axis1.encoder.pos_estimate
        axis0_vel = self.odrv.axis0.encoder.vel_estimate
        axis1_vel = self.odrv.axis1.encoder.vel_estimate
        return [axis0_pos, axis0_vel, axis1_pos, axis1_vel]




def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    odrive_controller = Odrive_ep()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    odrive_controller.get_logger().info(
        'Odrive controller Started with threads='+str(num_threads))
    
    executor = MultiThreadedExecutor(num_threads=num_threads)
    executor.add_node(odrive_controller)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        odrive_controller.destroy_node()

    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()
