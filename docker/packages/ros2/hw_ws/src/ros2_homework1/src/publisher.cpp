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
        RCLCPP_INFO(node->get_logger(), "Publishing: %d", msg.data); // Explain this line
        // Fill the missing line of code
        // Hint: execute the callback function under the "node" node
        

        loop_rate.sleep();
        count++;
    }
    
    rclcpp::shutdown();
    return 0;    
}