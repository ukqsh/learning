#include "ros/ros.h"
#include "geometry_msgs/Point.h"

#include <sstream>

int main(int argc, char **argv)
{
ros::init(argc,argv,"position_publisher");
ros::NodeHandle n;
ros::Publisher position_pub = n.advertise<geometry_msgs::Point>("pos",50);

double x = 0.0;
double y = 0.0;
double z = 0.0;
double v = 8.0;
ros::Time current_time, last_time;
current_time = ros::Time::now();
last_time = ros::Time::now();


ros::Rate = loop_rate(10);
int count = 0;
while (ros::ok())
{
    ros::spinOnce();
    current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    double delta_x = v * dt;
    x + = delta_x;
    geometry_msg::Point pos;
    pos.x = x;
    pos.y = y;
    pos.z =z;
    position_pub.publish(pos);
    last_time = current_time;
    
    loop_rate.sleep();
  }

}