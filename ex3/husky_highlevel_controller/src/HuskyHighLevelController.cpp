// HuskyHighLevelController.cpp

#include <algorithm>
#include <cmath>

#include "HuskyHighLevelController.hpp"

HuskyHighLevelController::HuskyHighLevelController(const ros::NodeHandle& handle) : m_NodeHandle(handle), m_ProcessVarTheta(0)
{
    // publishers for this controller
	m_VelocityPublisher = m_NodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    m_PillarMarkerPublisher = m_NodeHandle.advertise<visualization_msgs::Marker>("/pillar_marker", 100);

    // setup pillar marker
    SetupPillarMarker();

	// read in params
	if (!m_NodeHandle.getParam("control_gain", m_ControlGain)) {
	    ROS_ERROR("Param control_gain not found in config file");
	}

	if (!m_NodeHandle.getParam("thrust_speed", m_ThrustSpeed)) {
	    ROS_ERROR("Param thrust_speed not found in config file");
	}
}

// Pillar marker setup
void HuskyHighLevelController::SetupPillarMarker()
{
    m_PillarMarker.header.frame_id = "base_laser";
    m_PillarMarker.id = 0;
    m_PillarMarker.type = visualization_msgs::Marker::CUBE;
    m_PillarMarker.action = visualization_msgs::Marker::ADD;
    m_PillarMarker.pose.position.x = 1;
    m_PillarMarker.pose.position.y = 1;
    m_PillarMarker.pose.position.z = 1;
    m_PillarMarker.pose.orientation.x = 0.0;
    m_PillarMarker.pose.orientation.y = 0.0;
    m_PillarMarker.pose.orientation.z = 0.0;
    m_PillarMarker.pose.orientation.w = 1.0;
    m_PillarMarker.scale.x = 1;
    m_PillarMarker.scale.y = 1;
    m_PillarMarker.scale.z = 1;
    m_PillarMarker.color.a = 1.0;
    m_PillarMarker.color.r = 0.0;
    m_PillarMarker.color.g = 0.0;
    m_PillarMarker.color.b = 1.0;
}

HuskyHighLevelController::~HuskyHighLevelController() {}

// Read the scan and determine the pillar location
void HuskyHighLevelController::ReadScan(const sensor_msgs::LaserScan& scan)
{
	m_CurrentScan = scan;

	// determine the position of the pillar
	geometry_msgs::Twist twistToPillar;
	if (CalculatePillarTwist(twistToPillar))
	{
	    // publish the twist to move towards it
		m_VelocityPublisher.publish(twistToPillar);
    }
}

// Calculation of the twist to move towards the pillar
bool HuskyHighLevelController::CalculatePillarTwist(geometry_msgs::Twist& result)
{
	// determine the index of the min range
	int index = 0;
	std::vector<float>::iterator iter = std::min_element(m_CurrentScan.ranges.begin(), m_CurrentScan.ranges.end());

	// only process if valid range
	if (m_CurrentScan.range_min <= 0.0) {
	    m_ProcessVarTheta = 0;
	    return false;
	}

	if (iter != m_CurrentScan.ranges.end())
	{
		index = std::distance(m_CurrentScan.ranges.begin(), iter);

		// got an index, so get the angle
		m_ProcessVarTheta = m_CurrentScan.angle_increment * index  + m_CurrentScan.angle_min;

		// set the twist required to move towards the pillar (control angle towards 0)
		result.angular.z = m_ControlGain * (0 - m_ProcessVarTheta);
		result.linear.x = m_ThrustSpeed;

		// publish pillar marker location
		float x = m_CurrentScan.ranges.at(index) * cos(m_ProcessVarTheta);
		float y = m_CurrentScan.ranges.at(index) * sin(m_ProcessVarTheta);
		PublishPillarMarker(x, y);

		return true;
	}

	m_ProcessVarTheta = 0;
	return false;
}

// Marker for the pillar
void HuskyHighLevelController::PublishPillarMarker(float x, float y)
{
    m_PillarMarker.pose.position.x = x;
    m_PillarMarker.pose.position.y = y;
    m_PillarMarker.pose.position.z = 0;
    m_PillarMarker.header.stamp = ros::Time();

    m_PillarMarkerPublisher.publish(m_PillarMarker);
}

// Rad to deg
float HuskyHighLevelController::radiansToDegrees(float radians) const
{
    return radians * (180.0 / M_PI);
}
