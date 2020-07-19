#include <hellocm_msgs/msg/ext2_cm.hpp>
#include "rclcpp/rclcpp.hpp"
#include <chrono>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class SimplePublisher : public rclcpp::Node {
public:
  SimplePublisher() : Node("simple_publisher") {
    publisher_ = this->create_publisher<hellocm_msgs::msg::Ext2CM>("chatter");
    timer_ = this->create_wall_timer(
        500ms, std::bind(&SimplePublisher::timer_callback, this));
  }

private:
  void timer_callback() {
      auto msg = hellocm_msgs::msg::Ext2CM();
      long int cycleno = 20;
        
        //rclcpp::Clock currentTime =  rclcpp::Clock::now();
        //msg.header.stamp = currentTime;
        //msg.time = currentTime;
      msg.cycleno = cycleno;
      RCLCPP_INFO(this->get_logger(), "the cycleno is %ld",  msg.cycleno);
     // RCLCPP_INFO(this->get_logger(), "time from time is %f", msg->time.sec + 1e-9*msg->time.nanosec);
     // RCLCPP_INFO(this->get_logger(), "the time we get the pose is %f",  msg->header.stamp.sec + 1e-9*msg->header.stamp.nanosec);
      publisher_->publish(msg);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<hellocm_msgs::msg::Ext2CM>::SharedPtr publisher_;
    
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimplePublisher>());
  rclcpp::shutdown();
  return 0;
}
