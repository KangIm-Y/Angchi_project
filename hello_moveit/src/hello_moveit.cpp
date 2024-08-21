#include <memory>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Next step goes here
// Create the MoveIt MoveGroup Interface
using moveit::planning_interface::MoveGroupInterface;
auto move_group_interface = MoveGroupInterface(node, "indy_manipulator");


float x = 1.0;
float y = 1.2;
float z = 2.0;
float r = 0.0;
std::cout << "Enter the X: ";
std::cin >> x;
std::cout << "Enter the Y: ";
std::cin >> y;
std::cout << "Enter the Z: ";
std::cin >> z;
// Set a target Pose
auto target_pose = [=](){
  geometry_msgs::msg::Pose msg;
  msg.orientation.w = 1.0;
  msg.position.x = x;
  msg.position.y = y;
  msg.position.z = z;
  return msg;
}();
RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"target pose X: %.2f  Y: %.2f Z: %.2f", target_pose.position.x, target_pose.position.y, target_pose.position.z);
move_group_interface.setPoseTarget(target_pose);

// Create a plan to that target pose
auto const [success, plan] = [&move_group_interface]{
  moveit::planning_interface::MoveGroupInterface::Plan msg;
  auto const ok = static_cast<bool>(move_group_interface.plan(msg));
  return std::make_pair(ok, msg);
}();

// Execute the plan
std::cout << "Plan success. Move:1, cancel:0 \n Enter value : ";
std::cin >> r;

if(success && r > 0) {
  move_group_interface.execute(plan);
}
else if(success && r <= 0){
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Excute cancel!");
}
 else {
  RCLCPP_ERROR(logger, "Planing failed!");
}
  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
