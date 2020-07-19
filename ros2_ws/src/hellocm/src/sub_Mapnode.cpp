#include <memory>
#include <functional>
#include <string>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <hellocm_msgs/msg/command.hpp>
using std::placeholders::_1;
using namespace std::chrono_literals;


static double Steering;
static double VposX, VposY;


class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("ROS2mapnode")
    {
      subscription_ = this->create_subscription<hellocm_msgs::msg::Command>(
      "PoseStamped", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
      publisher_ = this->create_publisher<hellocm_msgs::msg::Command>("/hellocm/command", 10);    // CHANGE
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalSubscriber::timer_callback, this));
    }

  private:
    void topic_callback(const hellocm_msgs::msg::Command::SharedPtr msg) const
    { 
	  Steering = msg->steering;
	  VposX = msg->vpose.pose.position.x;
	  VposY = msg->vpose.pose.position.y;
      RCLCPP_INFO(this->get_logger(), "I heard the pose from the robot");
      RCLCPP_INFO(this->get_logger(), "the position(x,y,z) is %f , %f, %f", msg->vpose.pose.position.x, msg->vpose.pose.position.y, msg->vpose.pose.position.z);
      RCLCPP_INFO(this->get_logger(), "the orientation(x,y,z,w) is %f , %f, %f, %f", msg->vpose.pose.orientation.x, msg->vpose.pose.orientation.y, msg->vpose.pose.orientation.z, msg->vpose.pose.orientation.w);
      RCLCPP_INFO(this->get_logger(), "the time we get the pose is %f",  msg->vpose.header.stamp.sec + 1e-9*msg->vpose.header.stamp.nanosec);
      RCLCPP_INFO(this->get_logger(), "steering angle is %.3f",  msg->steering);
    }
    
    void timer_callback()
    {
      auto message = hellocm_msgs::msg::Command();

      message.steering = Steering;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.steering);
      publisher_->publish(message);

    }
    rclcpp::Subscription<hellocm_msgs::msg::Command>::SharedPtr subscription_;    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<hellocm_msgs::msg::Command>::SharedPtr publisher_;
    size_t count_;

};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
