#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Twist.h>

#include "turtlesim_autodraw/polygon.h"

const double speed = 3;
const double angular_speed = 3;

ros::Publisher publisher;


void draw_in_line(float side_length, geometry_msgs::Twist &vel_msg, ros::Publisher &pub)
{
	vel_msg.linear.x = speed; 
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = 0;

	double t0 = ros::Time::now().toSec();

	double distance_travelled = 0;

	while (distance_travelled < side_length) 
	{
		pub.publish(vel_msg);
		
		double t1 = ros::Time::now().toSec();
		distance_travelled = speed * (t1 - t0);
	}
	vel_msg.linear.x = 0;
	pub.publish(vel_msg);
}

void rotate(geometry_msgs::Twist &vel_msg, ros::Publisher &pub, int num)
{
	vel_msg.angular.z = angular_speed;

	double t0 = ros::Time::now().toSec();

	double angle_travelled = 0;

	const double turn_angle = M_PI - (M_PI * (1.0 - ( 2.0 / num)));

	while (angle_travelled < turn_angle)
	{
		pub.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		angle_travelled = angular_speed * (t1 - t0);
	}
	vel_msg.angular.z = 0;
	pub.publish(vel_msg);
}

bool handlepolygon(turtlesim_autodraw::polygon::Request &req, turtlesim_autodraw::polygon::Response &res)
{

	ROS_INFO("[draw_polygon_server] I just got incoming request!");

	geometry_msgs::Twist vel_msg;

	float side_length = req.sidelength;
	ROS_INFO_STREAM("[draw_polygon_server] req.sidelength=" << req.sidelength);

	int32_t rotations = req.rotations;
	ROS_INFO_STREAM("[draw_polygon_server] req.rotations=" << req.rotations);

    int32_t num = req.num;
    ROS_INFO_STREAM("[draw_polygon_server] req.num=" << req.num);

	int current_rotation_quarter = 0;
	while (current_rotation_quarter < rotations * num && ros::ok())
	{
		ROS_DEBUG_STREAM("[draw_polygon_server] current_rotation_quarter, rotations=" << current_rotation_quarter << ", " << rotations);

		draw_in_line(side_length, vel_msg, publisher);
		rotate(vel_msg, publisher, num);
		current_rotation_quarter++;
	}

	return true;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "draw_polygon_server");
	ros::NodeHandle nh;
	ROS_INFO("[draw_polygon_server] Startup");

	ros::Rate loop_rate(10);

	publisher = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
	ROS_INFO("[draw_polygon_server] Advertising publisher /turtle1/cmd_vel");

	ros::ServiceServer s = nh.advertiseService<turtlesim_autodraw::polygon::Request, turtlesim_autodraw::polygon::Response>("draw_polygon", handlepolygon);
	ROS_INFO("[draw_polygon_server] Advertising service server draw_polygon");

	while(ros::ok()) ros::spinOnce();

	return 0;
}
