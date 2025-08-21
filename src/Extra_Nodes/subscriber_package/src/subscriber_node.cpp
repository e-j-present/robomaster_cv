#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32_multi_array.hpp"

class SubscriberNode : public rclcpp::Node {
public:
    SubscriberNode() : Node("SubscriberNode") {
        subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>("int_array", 10, 
                        std::bind(&SubscriberNode::callback, this, std::placeholders::_1));
    }

private:
    void callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        std::vector<int> out = msg->data;
        RCLCPP_INFO(this->get_logger(), "Received: %d, %d, %d, %d, %d", out[0], out[1], out[2], out[3], out[4]);
    }
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubscriberNode>());
    rclcpp::shutdown();
    return 0;
}
