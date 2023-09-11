#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"

using std::placeholders::_1;

/**
 * @brief This is a class of node that converts the stroke to pwm signal
 * @author Chih Han Yang
 * 
 */
class StrokeToPWMNode : public rclcpp::Node
{
    public:
        /**
         * @brief Construct a new Stroke To PWM Node object
         * 
         */
        StrokeToPWMNode() : Node("stroke_to_pwm_node")
        {
            subscription_ = this->create_subscription<std_msgs::msg::Int32>(
                "pedal_stroke", 10, std::bind(&StrokeToPWMNode::topic_callback, this, _1)
            );
            publisher_ = this->create_publisher<std_msgs::msg::Int32>("pwm_signal", 10);

        }
    private:
        /**
         * @brief this is the callback function called as the subscriber recieved a message
         * 
         * @param msg 
         */
        void topic_callback(const std_msgs::msg::Int32 & msg) const
        {
            RCLCPP_INFO(this->get_logger(), "Stroke: %d", msg.data);
            auto pub_msg = std_msgs::msg::Int32();
            pub_msg.data = (int)((float)msg.data * (127.0/2047.0));
            RCLCPP_INFO(this->get_logger(), "PWM: %d", pub_msg.data);
            publisher_->publish(pub_msg);
        }
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StrokeToPWMNode>());
    rclcpp::shutdown();
    return 0;
}
