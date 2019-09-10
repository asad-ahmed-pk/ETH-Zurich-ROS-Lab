// HuskyHighLevelConttroller.cpp

#include "HuskyHighLevelController.hpp"

HuskyHighLevelController::HuskyHighLevelController(const ros::NodeHandle& handle) : m_NodeHandle(handle)
{}

HuskyHighLevelController::~HuskyHighLevelController() {}

// Read the scan
void HuskyHighLevelController::ReadScan(const sensor_msgs::LaserScan& scan)
{
	ROS_INFO("Smallest Range: %f", scan.range_min);
}
