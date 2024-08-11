#include <thread>
#include <memory>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "custom_interfaces/srv/position_service.hpp"
#include <signal.h>

using std::placeholders::_1;
using std::placeholders::_2;
using moveit::planning_interface::MoveGroupInterface;

bool moved_flag = true;

int r = 0;
float x = 0.0;
float y = 0.0;
float z = 0.0;
double ori_w = 0.0;
double ori_x = 0.0;
double ori_y = 0.7071231592334566;
double ori_z = -0.7070904020014414;


std::shared_ptr<MoveGroupInterface> move_group_interface;
rclcpp::Logger logger = rclcpp::get_logger("mani_moveit");

void signal_callback_handler(int signum) {
    std::cout << "Caught signal " << signum << std::endl;
    // Terminate program
    exit(signum);
}

bool PlanAndExecute() {
   auto target_pose = []{
        geometry_msgs::msg::Pose msg;
        msg.orientation.w = ori_w;
        msg.orientation.x = ori_x;
        msg.orientation.y = ori_y;
        msg.orientation.z = ori_z;
        msg.position.x = x;
        msg.position.y = y;
        msg.position.z = z;
        return msg;
    }();    
    RCLCPP_INFO(logger, "Target pose position :    X: %.2f  Y: %.2f Z: %.2f,      orientation :    X: %.2f  Y: %.2f Z: %.2f W: %.2f", target_pose.position.x, target_pose.position.y, target_pose.position.z, target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);
	
    
    move_group_interface->setPoseTarget(target_pose);
    //move_group_interface->setPoseTarget(x,y,z);
    //move_group_interface->setRPYTarget(-1.55039099, 0, 3.141592);

    // Create a plan to that target pose
    auto const [succeed, plan] = [&move_group_interface] {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface->plan(msg));
        return std::make_pair(ok, msg);
    }();

    if (succeed) {
        move_group_interface->execute(plan);
        RCLCPP_INFO(logger, "Execution!");
        return true; 
    }
    else {
        RCLCPP_ERROR(logger, "Planning failed!");
        return false;
    }
}

class MoveitServerNode : public rclcpp::Node {
public:
    MoveitServerNode() : Node("pos_server") {
        server_ = this->create_service<custom_interfaces::srv::PositionService>(
            "pos_srv",
            std::bind(&MoveitServerNode::callbackPos, this, _1, _2));
        RCLCPP_INFO(this->get_logger(), "Service server has been started.");
    }

private:
    void callbackPos(const custom_interfaces::srv::PositionService::Request::SharedPtr request,
                     const custom_interfaces::srv::PositionService::Response::SharedPtr response) {
        x = request->coordinate.x;
        y = request->coordinate.y;
        z = request->coordinate.z;
        moved_flag = false;

        response->success = PlanAndExecute();
    }

    rclcpp::Service<custom_interfaces::srv::PositionService>::SharedPtr server_;
};

int main(int argc, char *argv[]) {
    // Initialize ROS and create the Node
    signal(SIGINT, signal_callback_handler);
    rclcpp::init(argc, argv);

    // Create the MoveIt MoveGroup Interface
    auto const node = std::make_shared<rclcpp::Node>(
        "mani_moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    move_group_interface = std::make_shared<MoveGroupInterface>(node, "mani");

    auto mani_server = std::make_shared<MoveitServerNode>();

    move_group_interface->setMaxVelocityScalingFactor(1);
    move_group_interface->setMaxAccelerationScalingFactor(1);

    rclcpp::spin(mani_server);

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
