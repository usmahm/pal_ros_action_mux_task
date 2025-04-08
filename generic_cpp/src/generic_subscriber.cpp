#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/generic_subscription.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class GenericSubscriber : public rclcpp::Node
{
  public:
    GenericSubscriber(std::string topic_name) : Node("generic_subscriber"), topic_name_(topic_name)
    {
      timer_ = this->create_wall_timer(5s, std::bind(&GenericSubscriber::attempt_sub_all_msg_types, this));

      RCLCPP_INFO(this->get_logger(), "Generic Subscriber Initialized on topic: '%s'", topic_name_.c_str());
    }

  private:
  // [Fix] change function name to ...
    void attempt_sub_all_msg_types()
    {
      // get all topics and types map
      auto topics_details = this->get_topic_names_and_types();

      // search throuh to get topic name equals the specified topic
      auto topic_details = topics_details.find(topic_name_);

      // [FIX]: since we are only subscribing to one at a time because of restriction, remove the loop
      // and only subscribe to the first and most likely only type on that topic
      if (topic_details != topics_details.end() && !topic_details->second.empty()) {
        for (const auto &topic_type : topic_details->second) {
          // loop through all the message types on that topic name and subscibe to them
          RCLCPP_INFO(this->get_logger(), "Subscribing to topic '%s' with type: %s ", topic_name_.c_str(), topic_type.c_str());

          auto sub = this->create_generic_subscription(
            topic_name_,
            topic_type,
            rclcpp::QoS(10),
            std::bind(&GenericSubscriber::sub_cb, this, std::placeholders::_1, topic_type)
          );

          subscriptions_.push_back(sub);
        }

        timer_->cancel();
      } else {
        RCLCPP_INFO(this->get_logger(), "Could not find any publisher on topic: %s, retrying in 5s", topic_name_.c_str());
      }
    }

    void sub_cb(const std::shared_ptr<rclcpp::SerializedMessage> msg, std::string msg_type)
    {
      (void)msg;
      RCLCPP_INFO(this->get_logger(), "Received a message from topic '%s' with type: %s", topic_name_.c_str(), msg_type.c_str());
    }

    std::string topic_name_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<GenericSubscriber>("/buffer");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}