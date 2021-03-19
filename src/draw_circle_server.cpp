#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Twist.h>

#include "turtlesim_autodraw/circle.h"

ros::Publisher publisher;

void draw_circle(float speed, float radius, geometry_msgs::Twist &vel_msg, ros::Publisher &pub)
{
	vel_msg.linear.x = speed; 
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = speed/radius;

	while (ros::ok()) 
	{
		pub.publish(vel_msg);
	}
	vel_msg.linear.x = 0;	
	vel_msg.linear.z = 0;
	pub.publish(vel_msg);
}

bool handlecircle(turtlesim_autodraw::circle::Request &req, turtlesim_autodraw::circle::Response &res)
{

	ROS_INFO("[draw_circle_server] I just got incoming request!");
	
	geometry_msgs::Twist vel_msg;

	float speed = req.speed;
	ROS_INFO_STREAM("[draw_circle_server] req.speed=" << req.speed);

	float radius = req.radius;
	ROS_INFO_STREAM("[draw_circle_server] req.radius=" << req.radius);

	while (ros::ok())
    {
		draw_circle(speed, radius, vel_msg, publisher);
    }

	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "draw_circle_server");
	ros::NodeHandle nh;
	ROS_INFO("[draw_circle_server] Startup");

	ros::Rate loop_rate(10);

	publisher = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
	ROS_INFO("[draw_circle_server] Advertising publisher /turtle1/cmd_vel");

	ros::ServiceServer s = nh.advertiseService<turtlesim_autodraw::circle::Request, turtlesim_autodraw::circle::Response>("draw_circle", handlecircle);
	ROS_INFO("[draw_circle_server] Advertising service server draw_circle");

	while(ros::ok()) ros::spinOnce();

	return 0;
}