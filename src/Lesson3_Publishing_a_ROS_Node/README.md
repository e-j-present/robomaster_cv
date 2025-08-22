# Lesson Three: Publishing a ROS Node

In this lesson we will learn about publishing data in a ROS Node.

A node may publish data to any number of topics. After data is published to a topic, another node can subscribe to that topic to get the data sent by the original node.

![Single receiver topic diagram](https://docs.ros.org/en/humble/_images/Topic-SinglePublisherandSingleSubscriber.gif)

We will only focus on how to publish data in this lesson and in the next lesson we will discuss subscribing to topics.

First, create a new node. Name the node three_node and the package three_package. After creating it make the node spin. Refer back to lesson one and two for refrence.

At the top of three_node, add a new import by adding the following line.

```
#include "std_msgs/msg/int32_multi_array.hpp"
```

This imports a data type that can be published by a ROS publisher. The data type is simply an array of integers. ROS publishers can only publish messages and not raw data types. Thus, we must use the int32_multi_array because it is an std_msgs type, however there are also lots of other message types that can be published that are not int32_multi_array. 

Now, create a private section of the class. In this section we must first create our publisher. This is done with the following line.

```
rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
```

This creates a new publisher called `publisher_`, that is an Int32MultiArray data type.

Now we must initialize the publisher. Go to your constructor in the public part of your class and add the following line.

```
publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("int_array", 10);
```

This initializes the publisher and sets the topic that it will be publishing to, `"int_array"`. In order for a different node to get the data sent by this publisher, the node must subscribe to the `"int_array"` topic.

Next is where we actually feed the data to the publisher. In the private section of the class, create a void method called `callback` that takes in no parameters.

In this method, create a new Int32MultiArray, and call it `msg`.

```
std_msgs::msg::Int32MultiArray msg;
```

Now, set `msg.data` to be an array of 5 ints.

```
msg.data = {1, 2, 3, 4, 5};
```
For testing purposes, print out a message here so we know everytime something is published. Using RCLCPP_INFO(this->get_logger, "{Your Message}") is a safer way to print to the console from ROS nodes as opposed to using printf();

```
RCLCPP_INFO(this->get_logger(), "Publishing array of 5 ints");
```

Now, to publish the data pass `msg` into the publisher.

```
publisher_->publish(msg);
```

Now, everytime `callback()` is called, `msg` will be published to the topic `int_array`. For the purpose of this lesson, we want something to be published every second. In order to do this, we must use a timer that calls `callback()` every second.

First, create the timer next to where the publisher was created in the private part of the class.

```
rclcpp::TimerBase::SharedPtr timer_;
```

Then, initalize the timer to go call `callback()` every second in the public constructor.

```
timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&ThreeNode::callback, this));
```

Now, go to the root directory and build your package. To run, you will need two terminals open. Build your package. Go ahead and also build the package `subscriber_package`. Remember to sl after building.

In the first terminal, you will run your package.

```
ros2 run three_package three_node
```

This will begin publishing your array to the topic. You should see your print statement being printed out every second to the console.

In the second terminal, run the subscriber package.

```
ros2 run subscriber_package subscriber_node
```

This will subscribe to the `int_array` topic and begin printing out the array that you are publishing.

Congrats on finishing lesson three!