#include <memory>
#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <hellocm_msgs/msg/ext2_cm.hpp>
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<hellocm_msgs::msg::Ext2CM>(
      "/hellocm/cm2ext", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const hellocm_msgs::msg::Ext2CM::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "the cycleno is %ld", msg->cycleno);
      //RCLCPP_INFO(this->get_logger(), "time from time is %f", msg->time.sec + 1e-9*msg->time.nanosec);
      //RCLCPP_INFO(this->get_logger(), "the time we get the pose is %f",  msg->header.stamp.sec + 1e-9*msg->header.stamp.nanosec);
    }
    rclcpp::Subscription<hellocm_msgs::msg::Ext2CM>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}


