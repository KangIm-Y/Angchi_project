#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class ActiveGripperService : public rclcpp::Node
{
public:
    ActiveGripperService()
    : Node("active_gripper_service")
    {
        // 'ActiveGripper' 서비스 생성
        service_ = this->create_service<std_srvs::srv::SetBool>(
            "ActiveGripper", 
            std::bind(&ActiveGripperService::handle_service, this, _1, _2)
        );
    }

private:
    // 서비스 요청을 처리하는 콜백 함수
    void handle_service(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        // 요청 내용 출력
        RCLCPP_INFO(this->get_logger(), "Received request: %s", request->data ? "true" : "false");
        
        if (request->data) {
            // Gripper 활성화 로직
            RCLCPP_INFO(this->get_logger(), "Activating gripper...");
            response->success = true;
            response->message = "Gripper activated";
        } else {
            // Gripper 비활성화 로직
            RCLCPP_INFO(this->get_logger(), "Deactivating gripper...");
            response->success = true;
            response->message = "Gripper deactivated";
        }
    }

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
};

int main(int argc, char **argv)
{
    // ROS 2 초기화
    rclcpp::init(argc, argv);

    // 노드 실행
    rclcpp::spin(std::make_shared<ActiveGripperService>());

    // 종료 처리
    rclcpp::shutdown();
    return 0;
}
