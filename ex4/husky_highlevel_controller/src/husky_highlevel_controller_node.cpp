// husky_highlevel_controller.cpp

#include <ros/ros.h>
#include <iostream>
#include <string>

#include "HuskyHighLevelController.hpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "husky_highlevel_controller");
	ros::NodeHandle nodeHandle("~");

	HuskyHighLevelController controller { nodeHandle };

	// retrieve the params from the config file
	std::string topic { "scan" };
	if (!nodeHandle.getParam("topic", topic)) {
		ROS_ERROR("Could not find param for topic");
	}

	int queueSize = 1;
	if (!nodeHandle.getParam("queue_size", queueSize)) {
		ROS_ERROR("Could not find param for queue_size");
	}

	ros::Subscriber subscriber = nodeHandle.subscribe(topic, queueSize, &HuskyHighLevelController::ReadScan, &controller);

	ros::spin();

	return 0;
}
