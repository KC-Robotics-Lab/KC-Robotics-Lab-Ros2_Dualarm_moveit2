#include "rclcpp/rclcpp.hpp"
#include "inspire_hand_interface/msg/posact.hpp"

class PosSubscriber : public rclcpp::Node
{
public:
  PosSubscriber() : Node("getposact_subscriber")
  {
    subscription_ = this->create_subscription<inspire_hand_interface::msg::Posact>(
      "pos", 10, std::bind(&PosSubscriber::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const inspire_hand_interface::msg::Posact::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received: '%d %d %d %d %d %d'",
                msg->pos[0], msg->pos[1], msg->pos[2],
                msg->pos[3], msg->pos[4], msg->pos[5]);
  }

  rclcpp::Subscription<inspire_hand_interface::msg::Posact>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PosSubscriber>());
  rclcpp::shutdown();
  return 0;
}