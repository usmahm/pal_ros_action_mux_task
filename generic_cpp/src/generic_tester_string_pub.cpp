#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class GenericTesterStringPub : public rclcpp::Node
{
  public:
    GenericTesterStringPub(std::string topic_name)
      : Node("generic_tester_string_pub"), topic_name_(topic_name)
    {
      string_publisher_ = this->create_publisher<std_msgs::msg::String>(topic_name_, 10);

      timer_ = this->create_wall_timer(5s, std::bind(&GenericTesterStringPub::publish_messages, this));
    }
  private:
    void publish_messages()
    {
      std_msgs::msg::String string_msg;
      string_msg.data = "Generic subscriber working!";
      
      RCLCPP_INFO(this->get_logger(), "Publishing string message type of value: '%s'", string_msg.data.c_str());
      string_publisher_->publish(string_msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_publisher_;
    std::string topic_name_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<GenericTesterStringPub>("/buffer");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}