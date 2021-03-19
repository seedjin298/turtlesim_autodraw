#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>

#include "turtlesim_autodraw/polygon.h"


int main(int argc, char **argv)
{
	ros::init(argc, argv, "draw_polygon_client");
	ros::NodeHandle nh;
	ROS_INFO("[draw_polygon_client] Startup");

	// rospy.wait_for_service('draw_polygon')

	double side_length, rotations, num;

	try
	{
		std::string inputString;

        std::cout << "Enter a number : ";
        std::cin.clear();
        std::getline(std::cin, inputString);
        int num = std::stod(inputString);
        if (num <= 0)
            throw "num <= 0";
        ROS_DEBUG_STREAM("num=" << num);
        
        inputString = "";

		std::cout << "Enter Side length (0-5):";
		std::cin.clear(); 
		std::getline(std::cin, inputString); 
		double side_length = std::stod(inputString); 
		if (side_length <= 0)  
			throw "side_length <= 0";
		ROS_DEBUG_STREAM("side_length=" << side_length);

		inputString = "";

		std::cout << "Enter number of rotations : ";
		std::cin.clear(); 
		std::getline(std::cin, inputString); 
		int rotations = std::stoi(inputString); 
		if (rotations <= 0)						
			throw "rotations <= 0";
		ROS_DEBUG_STREAM("rotations=" << rotations);

		turtlesim_autodraw::polygon svcpolygon;
		ros::ServiceClient svcpolygonClient = nh.serviceClient<turtlesim_autodraw::polygon>("draw_polygon");
		
		svcpolygon.request.rotations = rotations;
		svcpolygon.request.sidelength = side_length;
        svcpolygon.request.num = num;
		
		svcpolygonClient.call(svcpolygon);
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