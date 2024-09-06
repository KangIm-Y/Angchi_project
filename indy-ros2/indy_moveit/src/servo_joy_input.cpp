
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/experimental/buffers/intra_process_buffer.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/qos_event.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <thread>

// We'll just set up parameters here
const std::string JOY_TOPIC = "/joy";
const std::string TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const std::string JOINT_TOPIC = "/servo_node/delta_joint_cmds";
const std::string EEF_FRAME_ID = "tcp";
const std::string BASE_FRAME_ID = "link0";

// Enums for button names -> axis/button array index

enum Axis
{
  LEFT_STICK_X = 0,
  LEFT_STICK_Y = 1,
  LEFT_TRIGGER = 2,
  RIGHT_STICK_X = 3,
  RIGHT_STICK_Y = 4,
  RIGHT_TRIGGER = 5,
  D_PAD_X = 6,
  D_PAD_Y = 7
};
enum Button
{
  A = 1,
  B = 0,
  X = 2,
  Y = 3,
  LEFT_BUMPER = 4,
  RIGHT_BUMPER = 5,
  CHANGE_VIEW = 15,
  MENU = 9,
  HOME = 8,
  LEFT_STICK_CLICK = 11,
  RIGHT_STICK_CLICK = 12
};
auto mode = 0;
auto grip = false;

// Some axes have offsets (e.g. the default trigger position is 1.0 not 0)
// This will map the default values for the axes
std::map<Axis, double> AXIS_DEFAULTS = { { LEFT_TRIGGER, 1.0 }, { RIGHT_TRIGGER, 1.0 } };
std::map<Button, double> BUTTON_DEFAULTS;
std::array<int, 3> button_state_previous_ = {0,0,0};

// To change controls or setup a new controller, all you should to do is change the above enums and the follow 2
// functions
/** \brief // This converts a joystick axes and buttons array to a TwistStamped or JointJog message
 * @param axes The vector of continuous controller joystick axes
 * @param buttons The vector of discrete controller button values
 * @param twist A TwistStamped message to update in prep for publishing
 * @param joint A JointJog message to update in prep for publishing
 * @return return true if you want to publish a Twist, false if you want to publish a JointJog
 */
bool convertJoyToCmd(const std::vector<float>& axes, const std::vector<int>& buttons,
                     std::unique_ptr<geometry_msgs::msg::TwistStamped>& twist,
                     std::unique_ptr<control_msgs::msg::JointJog>& joint)
{
  // Give joint jogging priority because it is only buttons
  // If any joint jog command is requested, we are only publishing joint commands
  if (buttons[A] || buttons[B] || buttons[X] || buttons[Y] || axes[D_PAD_X] || axes[D_PAD_Y])
  {
    // Map the D_PAD to the proximal joints
    joint->joint_names.push_back("joint1");
    joint->velocities.push_back(axes[D_PAD_X]);
    joint->joint_names.push_back("joint2");
    joint->velocities.push_back(axes[D_PAD_Y]);

    // Map the diamond to the distal joints
    joint->joint_names.push_back("joint5");
    joint->velocities.push_back(buttons[B] - buttons[X]);
    joint->joint_names.push_back("joint4");
    joint->velocities.push_back(buttons[Y] - buttons[A]);
    return false;
  }

  // The bread and butter: map buttons to twist commands
  twist->twist.linear.x = 0.2 * axes[RIGHT_STICK_X];
  twist->twist.linear.y = -0.2 * axes[RIGHT_STICK_Y];

  double lin_z_right = -0.2 * (axes[RIGHT_TRIGGER] - AXIS_DEFAULTS.at(RIGHT_TRIGGER));
  double lin_z_left = 0.2 * (axes[LEFT_TRIGGER] - AXIS_DEFAULTS.at(LEFT_TRIGGER));
  twist->twist.linear.z = lin_z_right + lin_z_left;

  twist->twist.angular.y = axes[LEFT_STICK_X];
  twist->twist.angular.x = axes[LEFT_STICK_Y];

  double roll_positive = -1 * buttons[RIGHT_BUMPER];
  double roll_negative = (buttons[LEFT_BUMPER]);
  twist->twist.angular.z = roll_positive + roll_negative;

  return true;
}

/** \brief // This should update the frame_to_publish_ as needed for changing command frame via controller
 * @param frame_name Set the command frame to this
 * @param buttons The vector of discrete controller button values
 */
void updateCmdFrame(std::string& frame_name, const std::vector<int>& buttons)
{
  if (buttons[CHANGE_VIEW] && frame_name == EEF_FRAME_ID)
    frame_name = BASE_FRAME_ID;
  else if (buttons[MENU] && frame_name == BASE_FRAME_ID)
    frame_name = EEF_FRAME_ID;
}

namespace moveit_servo
{
class JoyToServoPub : public rclcpp::Node
{
public:
  JoyToServoPub(const rclcpp::NodeOptions& options)
    : Node("joy_to_twist_publisher", options), frame_to_publish_(BASE_FRAME_ID)
  {

    
    // Setup pub/sub
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        JOY_TOPIC, rclcpp::SystemDefaultsQoS(),
        [this](const sensor_msgs::msg::Joy::ConstSharedPtr& msg) { return joyCB(msg); });

    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, rclcpp::SystemDefaultsQoS());
    joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(JOINT_TOPIC, rclcpp::SystemDefaultsQoS());
    joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("/joy_drive", rclcpp::SystemDefaultsQoS());

    // Create a service client to start the ServoNode
    servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
    servo_start_client_->wait_for_service(std::chrono::seconds(1));
    servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

    grip_start_client_ = this->create_client<std_srvs::srv::SetBool>("ActiveGripper");
    grip_start_client_->wait_for_service(std::chrono::seconds(1));
    
  }

  void joyCB(const sensor_msgs::msg::Joy::ConstSharedPtr& msg)
  {
    // Create the messages we might publish
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();
    auto joy_msg = std::make_unique<sensor_msgs::msg::Joy>();
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();

    rclcpp::Logger logger = rclcpp::get_logger("moveit_servo");


    if (msg -> buttons[9] == 1 && button_state_previous_[2] != 1)
    {
      if (mode == 0){
        RCLCPP_INFO(logger, "mode : drive");
        mode = 1;
      }
      else if(mode == 1)
      {
        RCLCPP_INFO(logger, "mode : manipulator");
        mode = 0;
      }
    }

    if (mode == 0){
    

      // This call updates the frame for twist commands
      updateCmdFrame(frame_to_publish_, msg->buttons);
      

      // Convert the joystick message to Twist or JointJog and publish
      if (convertJoyToCmd(msg->axes, msg->buttons, twist_msg, joint_msg))
      {
        // publish the TwistStamped
        twist_msg->header.frame_id = frame_to_publish_;
        twist_msg->header.stamp = this->now();
        twist_pub_->publish(std::move(twist_msg));
      }
      else
      {
        // publish the JointJog
        joint_msg->header.stamp = this->now();
        joint_msg->header.frame_id = "link0";
        joint_pub_->publish(std::move(joint_msg));
      }


      // if (msg->buttons[11] == 1 && button_state_previous_[0] != 1)
      // {
      //   RCLCPP_INFO(logger, "zero");
      //   request->pose = "zero";
      //   pose_start_client_->async_send_request(request);
      // }

      if (msg->buttons[8] == 1 && button_state_previous_[1] != 1)
      {
        if (grip == false)
        {
          RCLCPP_INFO(logger, "grip on");
          request->data = true;
          grip_start_client_->async_send_request(request);
          grip = true;
        }
        else if (grip == true)
        {
          RCLCPP_INFO(logger, "grip off");
          request->data = false;
          grip_start_client_->async_send_request(request);
          grip = false;
        }
      }

      
    }
    else if (mode == 1)
    {
      //RCLCPP_INFO(this->get_logger(), "Publishing message");
      joy_msg -> axes = msg -> axes;
      joy_msg -> buttons = msg -> buttons;
      joy_pub_-> publish(*joy_msg);
    }
    button_state_previous_ = {msg->buttons[11], msg->buttons[8], msg->buttons[9]};
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr grip_start_client_;

  std::string frame_to_publish_;
};  // class JoyToServoPub

}  // namespace moveit_servo

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(moveit_servo::JoyToServoPub)
