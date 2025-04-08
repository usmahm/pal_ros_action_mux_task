#include "rclcpp/rclcpp.hpp"
#include "action_interfaces/srv/string.hpp"
#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

class ActionClient : public rclcpp::Node
{
  public:
    ActionClient() : Node("action_client")
    {
      client_ = this->create_client<action_interfaces::srv::String>("action");
      action_subscriber_ = this->create_subscription<std_msgs::msg::String>("new_action", 10, std::bind(&ActionClient::action_cb, this, std::placeholders::_1));

      RCLCPP_INFO(this->get_logger(), "Action Client Ready. Listening on 'new_action' topic");
    }

    void action_cb(const std_msgs::msg::String &msg) {
      RCLCPP_INFO(this->get_logger(), "New action received from subscriber. Action: '%s'", msg.data.c_str());

      this->send_request(msg.data);
    }

    void send_request(std::string input)
    {
      auto request = std::make_shared<action_interfaces::srv::String::Request>();
      request->data = input;

      //[FIX]:  Change position of this check
      while(!client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
          return;
        }
        RCLCPP_INFO(this->get_logger(), "action service not available, waiting again...");
      }

      auto result = client_->async_send_request(
        request,
        std::bind(&ActionClient::handle_response, this, std::placeholders::_1));
    }

    void handle_response(rclcpp::Client<action_interfaces::srv::String>::SharedFuture future)
    {
      auto response = future.get();
      RCLCPP_INFO(this->get_logger(), "Output: %s", response->output.c_str());
    }
  
  private:
    rclcpp::Client<action_interfaces::srv::String>::SharedPtr client_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr action_subscriber_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<ActionClient>());

  rclcpp::shutdown();
  return 0;
}