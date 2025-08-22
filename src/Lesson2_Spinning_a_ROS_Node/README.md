# Lesson Two: Spinning a ROS Node

In this lesson we will learn what spinning a ROS Node is and how to do it.

You may have noticed that after running the ROS Node in lesson one it terminated instantly after printing its hello world output. However, we want our nodes to run infinitely. To do this we must spin the nodes.

First, create a new node in this folder. Name the node `two_node` and the package `two_package`. Refer back to Lesson One on how to do this if needed.

We must use external libraries in this lesson. The following steps will go through how to add these libraries to the code and add the correct dependencies for them so the compiler recognizes them.

To start, add a dependency for rclcpp at the top of your two_node file.

```
#include <rclcpp/rclcpp.hpp>
```

Now, go into your cmake file and add the following two lines just below the existing dependency `find_package(ament_cmake REQUIRED)`.

```
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
```

After, still in the cmake file, add the dependencies to your target after `add_executable(two_node src/two_node.cpp)` with the following line.

```
ament_target_dependencies(yourname_node rclcpp std_msgs)
```

Now, go into your xml file and add the following two dependencies after the line `<buildtool_depend>ament_cmake</buildtool_depend>` .

```
<depend>rclcpp</depend>
<depend>std_msgs</depend>
```

Now you should have imported rclcpp and taken care of all dependencies relating to it.

Go back into the two_node file and create a class titled `TwoNode` that extends `rclcpp::Node`.

Now, move the hello world print statement from the main method inside the constructor of your node.

```
class Lesson2Node : public rclcpp::Node {
public:
  Lesson2Node() : Node("Lesson2Node") {
    printf("hello world Lesson2_Package package\n");
  }
};
```

Inside the main method, add the following line to initialize the ROS environment inside your node.

```
rclcpp::init(argc, argv);
```

After that line add the following line to spin your node infinitly so that it will continuously run until it is forced to stop.

```
rclcpp::spin(std::make_shared<TwoNode>());
```

Finally, add the following line last, so that the node can stop spinning and shut down when needed.

```
rclcpp::shutdown();
```

Now, try building and running the node. You should see that the same hello world text from before prints out, but the node doesn't terminate after printing once. When you wish to stop the node, press `ctrl + c` in the terminal to terminate it.

Congrats on finishing lesson two!