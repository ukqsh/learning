#include "ros/ros.h"
//#include "Log.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include <hellocm_msgs/Ext2CM.h>
#include <hellocm_msgs/CM2Ext.h>
#include <hellocm_msgs/Command.h>

#if 0
# define LOG(frmt, ...)  printf(frmt "\n", ##__VA_ARGS__)
#else
# define LOG ROS_INFO
#endif

static double DeltaAng;
static double DevAngToRoute;
static double DevDistToRoute, AbsDevDistToRoute;
static double velocity;
static int vz;
static double steering_angle;
static double brake;
static double gas;
static double positionX, positionY, positionZ;
static double last_positionX, last_positionY, last_positionZ;
static double yaw, roll, pitch;
static double SimTime;

struct Vehicleposition{
	double x;
	double y;
};

struct Vehicleorientation{
	double x;
	double y;
	double z;
	double w;
};

void mapCallback(const hellocm_msgs::CM2Ext::ConstPtr &msg)
{

    //lateral_error = msg->tRoad;
    DeltaAng = msg->DeltaAng;
    DevAngToRoute = msg->DevAngToRoute;
    DevDistToRoute = msg->DevDistToRoute;
    AbsDevDistToRoute = std::abs(DevDistToRoute);
    velocity = msg->velocity;
    brake =  msg-> Brake;
    positionX = msg-> PrevViewPointX;
    positionY = msg-> PrevViewPointY;
    last_positionX = msg->PrevViewPointLastX;
    last_positionY = msg->PrevViewPointLastY;
    yaw = msg->yaw;
    roll = msg -> roll;
    pitch = msg ->pitch;
    SimTime = msg->time.toSec();
    //DevAngToRoute = vz * DevAngToRoute;
    if  (std::sin(DeltaAng) < 0)
        vz = -1;
    else
        vz = 1;
        
    
    

    LOG("CarMaker Node is in cycle %lu, Time=%.3fs, Velocity=%.3fm/s, Steering Angle = %.3f, delta angle = %.3f,  Gas = %.3f, Brake = %.3f,  DevAngToRoute = %.3f,  deviation =%.3fm, PositionX = %.3f, PositionY = %.3f, VPositionX = %.3f, VPositionY = %.3f, LastPositionX = %.3f, LastPositionY = %.3f, Stamp=%.3fs, SeqID=%d, vz=%d",
	   
	    msg->cycleno, msg->time.toSec(), msg->velocity, msg->SteeringAng, msg->DeltaAng, msg-> Gas, msg-> Brake, msg->DevAngToRoute, DevDistToRoute, msg->PrevViewPointX, msg->PrevViewPointY, msg->VehiclePosX, msg->VehiclePosY, msg->PrevViewPointLastX, msg->PrevViewPointLastY, msg->header.stamp.toSec(), msg->header.seq, vz);
}

Vehicleposition GetVehiclePos(double positionX, double positionY, double last_positionX, double last_positionY, double DevDistToRoute,double previewed_distance){
	struct Vehicleposition Vp;
	double x,y,n,x_u,y_u;
	if(positionX - last_positionX == 0 && positionY - last_positionY == 0){
		x = positionX;
		y = positionY;
	}
	else{
		x = (positionX - last_positionX);
		y = (positionY - last_positionY);
		
		std::cout << "x = " << x << "\n";
		std::cout << "y = " << y << "\n";
		n = std::sqrt(x*x + y*y);
		x_u = x/n;
		y_u = y/n;
		x = positionX - x_u * previewed_distance;
		y = positionY - y_u * previewed_distance; 
		x = x + y_u * DevDistToRoute;
		y = y - x_u * DevDistToRoute;
		
		
	}
	Vp.x = x;
	Vp.y = y;
	return Vp;
}

Vehicleorientation GetVehicleOrien(double yaw, double roll, double pitch){
	using namespace std;
	struct Vehicleorientation Vo;
	Vo.w = cos(roll/2)*cos(pitch/2)*cos(yaw/2)+sin(roll/2)*sin(pitch/2)*sin(yaw/2);
	Vo.x = sin(roll/2)*cos(pitch/2)*cos(yaw/2)-cos(roll/2)*sin(pitch/2)*sin(yaw/2);
	Vo.y = cos(roll/2)*sin(pitch/2)*cos(yaw/2)+sin(roll/2)*cos(pitch/2)*sin(yaw/2);
	Vo.z = cos(roll/2)*cos(pitch/2)*sin(yaw/2)-sin(roll/2)*sin(pitch/2)*cos(yaw/2);
	
	return Vo;
}


double calc_steering_angle(double DeltaAng, int vz, double DevDistToRoute, double velocity){
	double steering_ang;
	if(-DevDistToRoute > 20){
		steering_ang =   2.0;
		//gas = 1.0;
	}
	else if(-DevDistToRoute < 10){
		steering_ang =  -1  * 2.0;
		//gas = 0.2;
	}
	else{
		steering_ang =  (1 * DeltaAng + std::atan(10  * vz * DevDistToRoute/ (velocity + 0.1)))/3;//Stanley Controller
		//gas= 1.0;
	}
	return steering_ang;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "publisher");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/hellocm/cm2ext", 10, mapCallback);
    
    ros::Publisher chatter_pub = n.advertise<hellocm_msgs::Command>("PoseStamped", 1000);
    
    
    ros::Rate loop_rate(10);

    double orientationX, orientationY, orientationZ, orientationW;
    double previewed_distance = 0.1;
    
    while (ros::ok())
    {

        hellocm_msgs::Command msg; 

        //assign value to poseStamped

            //First assign value to "header".
        ros::Time currentTime = ros::Time::now();
        msg.vpose.header.stamp = currentTime;

            //Then assign value to "pose", which has member position and orientation
        Vehicleposition VPos = GetVehiclePos(positionX, positionY, last_positionX, last_positionY, AbsDevDistToRoute, previewed_distance);
        msg.vpose.pose.position.x = VPos.x;
        msg.vpose.pose.position.y = VPos.y;
        msg.vpose.pose.position.z = 0;

        Vehicleorientation Vo = GetVehicleOrien(yaw, roll, pitch);
        msg.vpose.pose.orientation.x = Vo.x;
        msg.vpose.pose.orientation.y = Vo.y;
        msg.vpose.pose.orientation.z = Vo.z;
        msg.vpose.pose.orientation.w = Vo.w;
        
        double steering_angle = calc_steering_angle(DeltaAng, vz, DevDistToRoute, velocity);
        
        if (SimTime == 0)
			msg.steering = 0;
	    else
			msg.steering = steering_angle;

        
        ROS_INFO("we publish the robot's position and orientaion"); 
        ROS_INFO("the position(x,y,z) is %f , %f, %f", msg.vpose.pose.position.x, msg.vpose.pose.position.y, msg.vpose.pose.position.z);
        ROS_INFO("the orientation(x,y,z,w) is %f , %f, %f, %f", msg.vpose.pose.orientation.x, msg.vpose.pose.orientation.y, msg.vpose.pose.orientation.z, msg.vpose.pose.orientation.w);
        ROS_INFO("the time we get the pose is %f",  msg.vpose.header.stamp.sec + 1e-9*msg.vpose.header.stamp.nsec);
        ROS_INFO("steering angle is %.3f",  msg.steering);
        
    
     chatter_pub.publish(msg);
     ros::spinOnce();
     loop_rate.sleep();
}
  ros::spin();

  return 0;
}
