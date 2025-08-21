#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32_multi_array.hpp"

class PublisherNode : public rclcpp::Node {
public:
  PublisherNode() : Node("PublisherNode") {
    publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("int_array", 10);
    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&PublisherNode::callback, this));
  }

private:
  void callback() {
      std_msgs::msg::Int32MultiArray msg;
      msg.data = {1, 2, 3, 4, 5};
      RCLCPP_INFO(this->get_logger(), "Publishing array of 5 ints");
      publisher_->publish(msg);
  }
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  (void) argc;
  (void) argv;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PublisherNode>());
  rclcpp::shutdown();
  return 0;
}