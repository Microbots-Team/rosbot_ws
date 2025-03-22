#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class StepperPublisher : public rclcpp::Node {
public:
  StepperPublisher() : Node("stepper_publisher") {
    publisher_ = this->create_publisher<std_msgs::msg::String>("stepper_command", 10);
  }

  void publish_command(const std::string &command) {
    auto message = std_msgs::msg::String();
    message.data = command;
    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Publishing: %s", command.c_str());
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StepperPublisher>();

  // Example: Send commands
  node->publish_command("w");  // Move forward
  rclcpp::sleep_for(std::chrono::seconds(2));
  node->publish_command("s");  // Stop

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
