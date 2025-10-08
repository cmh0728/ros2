#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class BasicPublisher : public rclcpp::Node
{
public:
  BasicPublisher()
  : Node("basic_publisher"), count_(0)
  {
    // 퍼블리셔 생성: 1) 메시지 타입, 2) 토픽 이름, 3) 큐 크기
    publisher_ = this->create_publisher<std_msgs::msg::String>("greetings", 10);

    // 타이머 설정: 0.5초마다 publish_callback 함수를 호출
    timer_ = this->create_wall_timer(500ms, std::bind(&BasicPublisher::publish_callback, this));

    RCLCPP_INFO(this->get_logger(), "basic_publisher 노드가 시작되었습니다.");
  }

private:
  void publish_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "안녕하세요! 카운트: " + std::to_string(count_++);

    // 생성한 메시지를 토픽으로 발행
    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "메시지 발행: '%s'", message.data.c_str());
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  // ROS 2 초기화; rclcpp::init은 ROS 2 실행을 위한 기본 설정을 완료
  rclcpp::init(argc, argv);

  // rclcpp::spin은 노드가 종료될 때까지 콜백을 처리하며 실행 상태 유지
  rclcpp::spin(std::make_shared<BasicPublisher>());
  rclcpp::shutdown();
  return 0;
}
