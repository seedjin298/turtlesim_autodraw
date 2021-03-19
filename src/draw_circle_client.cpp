#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>

#include "turtlesim_autodraw/circle.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "draw_circle_client");
	ros::NodeHandle nh;
	ROS_INFO("[draw_circle_client] Startup");

	// rospy.wait_for_service('draw_circle')

	double speed, radius;

	try
	{
		std::string inputString;

		std::cout << "Enter Radius (0 - 2.5):";
		std::cin.clear(); 
		std::getline(std::cin, inputString); 
		double radius = std::stod(inputString); 
		if (radius <= 0)  
			throw "radius <= 0";
		ROS_DEBUG_STREAM("radius=" << radius);

		inputString = "";

		std::cout << "Enter Speed : ";
		std::cin.clear(); 
		std::getline(std::cin, inputString); 
		double speed = std::stoi(inputString); 
		if (speed <= 0)						
			throw "speed <= 0";
		ROS_DEBUG_STREAM("speed=" << speed);

		turtlesim_autodraw::circle svccircle;
		ros::ServiceClient svccircleClient = nh.serviceClient<turtlesim_autodraw::circle>("draw_circle");
		
		svccircle.request.radius = radius;
		svccircle.request.speed = speed;
		
		svccircleClient.call(svccircle);
	}
	catch (const ros::Exception e)
	{
		ROS_ERROR_STREAM("Service call failed: " << e.what());
	}
	catch (const char* s)
	{
		ROS_ERROR_STREAM(s);
	}
}