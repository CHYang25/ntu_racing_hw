# ROS2 Homework 2
## Part 1
### Question 1-1
ROS2 topic communication is a special term of inter-process communication. It's like a communication tunnel for two nodes(two processes) in the ROS2 graph to send or to recieve messages, such that they can react according to them. Graphically speaking, topics are like edges between two nodes. There're also 3 other communication methods for ROS2 nodes, and topic is the easiest model.

### Question 1-2
```
#include "rclcpp/rclcpp.hpp"
// Fill the missing line of code
// Hint: include the ROS2 integer 32 bit library for messaging
#include "std_msgs/msg/int32.hpp"

int main(int argc, char** argv)
{
    // parse the command line arguments such as node names, or package names, or executable names
    rclcpp::init(argc, argv); // Explain this line
    // this creates an node called "publisher_node"
    auto node = rclcpp::Node::make_shared("publisher_node"); // Explain this line
    // this creates a topic which takes std_msgs::msg::Int32 type, 
    // and topic's name is "count_values", and the message backup queue size is set to 10
    auto publisher = node->create_publisher<std_msgs::msg::Int32>("count_values", 10); // Explalin this line
    
    // 
    rclcpp::Rate loop_rate(1);
        
    int count = 1;
    while (rclcpp::ok() && count <= 5)
    {
        auto msg = std_msgs::msg::Int32();
        // Fill the missing line of code
        // Hint: makes the content of "msg" to be the value of "count" variable
        msg.data = count;

        publisher->publish(msg);
        // This line is like fprintf function, which publish the message to the node's logger
        RCLCPP_INFO(node->get_logger(), "Publishing: %d", msg.data); // Explain this line
        // Fill the missing line of code
        // Hint: execute the callback function under the "node" node

        loop_rate.sleep();
        count++;
    }
    
    rclcpp::shutdown();
    return 0;    
}
```

## Part 2
After finishing ```stroke_to_pwm_node.cpp```, colcon build it and source the setup file. After that, 
1. In the first terminal, run
```
ros2 run ros2_homework2 stroke_to_pwm_node
```
2. In the second terminal, run
```
ros2 run ros2_homework2 motor_node
```
3. In the third terminal, run
```
ros2 bag record /motor_node/motor_revs
```
4. In the forth terminal, run
```
rqt_graph
```
5. In the fifth terminal, run
```
cd ws/src/hw_ws/src/ros2_homework2/bag_files
ros2 bag play rosbag2_2023_08_22-09_00_33
```
Then, everything would start running :D
