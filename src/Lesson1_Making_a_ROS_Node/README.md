# Lesson One: Creating a ROS Node

Welcome: In this lesson you are going to learn to make a ros node. For the sake of this onboarding you are going to make the node in the current lesson folder, so in this case Lesson1_Making_a_ROS_Node directory, but note that normally when you make a ros node, you want to make in the /Robomaster_CV/src folder. 

Run this command to switch into the correct directory.

```
cd src/Lesson1_Making_a_ROS_Node/
```
Now you should be in the Lesson1_Making_a_ROS_Node directory. To create a node, you will need to run the following template code in your console: (remove the brackets and replace with whatever is inside of it with the names you want)

```
ros2 pkg create --build-type ament_cmake --node-name {yourname_node} {yourname_package}
```

For this exercise, create a node called one_node, and a package called one_package. 


You should now see a new folder with the name one_package inside the Lesson1_Making_a_ROS_Node directory.


Now, we are going to learn to run the node. First step is to cd into the parent directory. You can do this by running the following command in the terminal.

```
cd /robomaster_cv
```

The first step is to build the package. To build a package run the two commands below :

```
build {package name}
sl
```

Note: Always remember to `sl` after you build.

Make sure the build was successful after running the build command. It should say summary: 1 package finished


Next to run the node, run the following command: 

```
ros2 run {package name} {node name}
```

If the node was ran correcly, you should see the line `hello world one_package package` printed in the terminal.


Check out the solutions folder to compare your code with the solution code.


Congrats on finishing lesson one!