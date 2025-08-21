# Lesson Four: Subscribing From ROS Nodes

In this lesson, we will learn about subscribing to topics.

In the last lesson, we learned about how to publish messages to topics. Now we will learn how to get the messages.

Begin by creating a new node. Title the node `four_node` and the package `four_package`. Also, make the node spin.

At the top of four_node, import the int32_multi_array data type with the following line.

```
#include "std_msgs/msg/int32_multi_array.hpp"
```

Now, add a private section to the class. In this section, create the subscriber with the following line.

```
rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
```

This creates a new subscriber of the type Int32MultiArray. Go to your public constructor initialize the subscriber with the following line.

```
subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>("int_array", 10, std::bind(&FourNode::callback, this, std::placeholders::_1));
```

This initializes the subscriber to get Int32MultiArray messages from the topic `int_array`. It also calls the callback method everytime the subscription is created. Now we must create the callback method.

Go into the private part of the class and create a new void method called `callback`. However, this time the method will take in the parameter `const std_msgs::msg::Int32MultiArray::SharedPtr msg`. This lets it recive Int32MultiArray messages.

Now, we just need to parse the message and get the array. There are different ways to do this, but a simple way is with the following lines. Put these inside the callback method.

```
std::vector<int> out = msg->data;
RCLCPP_INFO(this->get_logger(), "Received: %d, %d, %d, %d, %d", out[0], out[1], out[2], out[3], out[4]);
```

This just puts the data from msg into a vector of integers and prints the five integers out.

To run this, we must do the same thing as in lesson three with the two terminals. This time, build both this package and the package you created in lesson 3. Then run each node in a seperate terminal. You should see the publishing message being printed out in one terminal and the integers from the array being printed in the other terminal.

Congrats on finishing lesson four!