#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h" 
#include <hellocm_msgs/cntrl_cmd.h>

void chatterCallback(const hellocm_msgs::cntrl_cmd::ConstPtr & msg) //Note it is geometry_msgs::PoseStamped, not std_msgs::PoseStamped
{
    ROS_INFO("I heard the pose from the robot"); 
    ROS_INFO("the steering and gas is %f , %f", msg->steering, msg->throttle);


    std::cout<<"\n \n"<<std::endl; //add two more blank row so that we can see the message more clearly
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/control_comand", 10, chatterCallback);

    ros::spin();

    return 0;
}
