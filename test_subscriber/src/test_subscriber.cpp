#include "test_subscriber.hpp"

void TestSubscriber::topic_callback_(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "I heard: %s", msg->data.c_str());
}

TestSubscriber::TestSubscriber() : Node("test_subscriber_note")
{
  subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic_test",
      std::bind(&TestSubscriber::topic_callback_, this, std::placeholders::_1));
}