#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32_multi_array.hpp"

class Lesson4Node : public rclcpp::Node {
public:
    Lesson4Node() : Node("Lesson4Node") {
        subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>("int_array", 10, 
                        std::bind(&Lesson4Node::callback, this, std::placeholders::_1));
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
    rclcpp::spin(std::make_shared<Lesson4Node>());
    rclcpp::shutdown();
    return 0;
}