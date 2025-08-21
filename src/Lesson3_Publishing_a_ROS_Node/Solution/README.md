# Lesson 3 Solution

Make sure you are in the root directory.

Run these commands to run the solution code:

In your first terminal.

```
build Lesson3_Package
sl
ros2 run Lesson3_Package Lesson3_Node
```

In your second terminal.

```
build subscriber_package
sl
ros2 run subscriber_package subscriber_node
```

Your first terminal should print `Publishing array of 5 ints` every second.

Your second terminal should print the array every second.