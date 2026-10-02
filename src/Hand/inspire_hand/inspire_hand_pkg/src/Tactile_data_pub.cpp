#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <vector>

class AggregatorNode : public rclcpp::Node
{
public:
    AggregatorNode() : Node("aggregator_node")
    {
        sub1_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
            "tactile/indextip", 10,
            std::bind(&AggregatorNode::callback1, this, std::placeholders::_1));

        sub2_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
            "tactile/indexnail", 10,
            std::bind(&AggregatorNode::callback2, this, std::placeholders::_1));

        sub3_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
            "tactile/indexpad", 10,
            std::bind(&AggregatorNode::callback3, this, std::placeholders::_1));

        pub_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("aggregated_topic", 10);
    }

private:
    void callback1(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
    {
        buffer_[0] = msg->data;  // topic1의 데이터는 0번
        received1_ = true;
        try_publish();
    }

    void callback2(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
    {
        buffer_[1] = msg->data;  // topic2의 데이터는 1번
        received2_ = true;
        try_publish();
    }

    void callback3(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
    {
        buffer_[2] = msg->data;  // topic2의 데이터는 1번
        received3_ = true;
        try_publish();
    }

    void try_publish()
    {
        if (received1_ && received2_ && received3_)
        {
            auto out_msg = std_msgs::msg::UInt16MultiArray();
            // 두 개 벡터를 연결
            out_msg.data.reserve(buffer_[0].size() + buffer_[1].size() + buffer_[2].size());
            out_msg.data.insert(out_msg.data.end(), buffer_[0].begin(), buffer_[0].end());
            out_msg.data.insert(out_msg.data.end(), buffer_[1].begin(), buffer_[1].end());
            out_msg.data.insert(out_msg.data.end(), buffer_[2].begin(), buffer_[2].end());

            // RCLCPP_INFO(this->get_logger(), "Publishing aggregated data of size: %zu", out_msg.data.size());

            pub_->publish(out_msg);

            received1_ = false;
            received2_ = false;
            received3_ = false;
        }
    }

    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr sub1_;
    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr sub2_;
    rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr sub3_;
    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub_;

    std::array<std::vector<uint16_t>, 3> buffer_;  // 각 토픽의 데이터를 저장
    bool received1_ = false;
    bool received2_ = false;
    bool received3_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AggregatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}