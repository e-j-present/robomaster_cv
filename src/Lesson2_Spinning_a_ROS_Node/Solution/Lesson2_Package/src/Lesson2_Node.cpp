#include <cstdio>
#include <rclcpp/rclcpp.hpp>

class Lesson2Node : public rclcpp::Node {
public:
  Lesson2Node() : Node("Lesson2Node") {
    printf("hello world Lesson2_Package package\n");
  }
};

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Lesson2Node>());
  rclcpp::shutdown();

  return 0;
}
