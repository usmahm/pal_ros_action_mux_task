#include "rclcpp/rclcpp.hpp"
#include "action_interfaces/srv/string.hpp"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;


class ActionServer : public rclcpp::Node
{
  public:
    ActionServer() : Node("action_server")
    {
      service_ = this->create_service<action_interfaces::srv::String>(
        "action",
        std::bind(&ActionServer::action_delay_cb, this, std::placeholders::_1, std::placeholders::_2)
      );

      RCLCPP_INFO(this->get_logger(), "Action Service Ready");
    }

  private:
    void action_delay_cb(const std::shared_ptr<action_interfaces::srv::String::Request> request, std::shared_ptr<action_interfaces::srv::String::Response> response)
    {
      if (timer_) 
      {
        RCLCPP_INFO(this->get_logger(), "Cancelling previous goal");
        timer_->cancel();
      }

      RCLCPP_INFO(this->get_logger(), "Scheduling new future goal. Goal: '%s'", request->data.c_str());
      timer_ = this->create_wall_timer(5s, [this, request]() {
        this->action_cb(request);

        timer_->cancel();
      });

      response->output =  "Requested goal scheduled. Goal: '" + request->data + "'";
    }
    
    void action_cb(const std::shared_ptr<action_interfaces::srv::String::Request> request)
    {    
      RCLCPP_INFO(this->get_logger(), "Goal Activated: '%s'", request->data.c_str());
    }
    
    rclcpp::Service<action_interfaces::srv::String>::SharedPtr service_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Action Service Ready");
  
  rclcpp::spin(std::make_shared<ActionServer>());

  rclcpp::shutdown();
}