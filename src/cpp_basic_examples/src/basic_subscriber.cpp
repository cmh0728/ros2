#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class BasicSubscriber : public rclcpp::Node
{
public:
  BasicSubscriber()
  : Node("basic_subscriber")
  {
    // 서브스크라이버 생성: 토픽 이름과 콜백을 등록
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "greetings",
      10,
      std::bind(&BasicSubscriber::topic_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "basic_subscriber 노드가 시작되었습니다.");
  }

private:
  void topic_callback(const std_msgs::msg::String & msg) const
  {
    // 토픽으로부터 받은 메시지 출력
    RCLCPP_INFO(this->get_logger(), "메시지 수신: '%s'", msg.data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BasicSubscriber>());
  rclcpp::shutdown();
  return 0;
}
