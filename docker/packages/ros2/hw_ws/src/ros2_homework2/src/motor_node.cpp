#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"

#include <cmath>

class MotorNode : public rclcpp::Node
{
public:
	MotorNode() : Node("motor_node")
	{
		subscription_ = this->create_subscription<std_msgs::msg::Int32>(
			"pwm_signal", 10, std::bind(&MotorNode::pwmSignalCallback, this, std::placeholders::_1));
		publisher_ = this->create_publisher<std_msgs::msg::Float32>("motor_revs", 10);
	}
private:
	void pwmSignalCallback(const std_msgs::msg::Int32::SharedPtr msg)
	{
		// Using a linear motor transfer function for example
		float pwm_value = static_cast<float>(msg->data);
		float motor_revs = pwmToRevs(pwm_value);

		auto motor_revs_msg = std_msgs::msg::Float32();
		motor_revs_msg.data = motor_revs;
		publisher_->publish(motor_revs_msg);
	}

	float pwmToRevs(float pwm_value)
	{
		// Actual motor transfer function
		float k = 10.0; // Motor constant
		float T = 0.13; // Time constant
		float normalized_signal = static_cast<float>(pwm_value) / 255.0;
		float speed = k * (1.0 - exp(-normalized_signal / T));
		return speed; // simple example
	}

	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MotorNode>());
	rclcpp::shutdown();
	return 0;
}
