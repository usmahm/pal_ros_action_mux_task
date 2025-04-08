#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

using namespace std::chrono_literals;

class GenericTesterIntPub : public rclcpp::Node
{
  public:
    GenericTesterIntPub(std::string topic_name)
      : Node("generic_tester_int_pub"), topic_name_(topic_name)
    {
      int_publisher_ = this->create_publisher<std_msgs::msg::Int64>(topic_name_, 10);

      timer_ = this->create_wall_timer(5s, std::bind(&GenericTesterIntPub::publish_messages, this));
    }
  private:
    void publish_messages()
    {
      std_msgs::msg::Int64 int_msg;
      int_msg.data = 30;
      
      RCLCPP_INFO(this->get_logger(), "Publishing integer message type of value: '%ld'", int_msg.data);
      int_publisher_->publish(int_msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr int_publisher_;
    std::string topic_name_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<GenericTesterIntPub>("/buffer");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}