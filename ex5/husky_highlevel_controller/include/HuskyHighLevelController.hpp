/*
 * HuskyHighLevelController.hpp
 *
 *  Created on: Sep 1, 2019
 *      Author: Asad Ahmed
 */

#ifndef SRC_HUSKYHIGHLEVELCONTROLLER_HPP_
#define SRC_HUSKYHIGHLEVELCONTROLLER_HPP_

#include <memory>

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>

class HuskyHighLevelController
{
	public:
		HuskyHighLevelController(const ros::NodeHandle& handle);
		~HuskyHighLevelController();

		void ReadScan(const sensor_msgs::LaserScan& scan);

	private:
		bool CalculatePillarTwist(geometry_msgs::Twist& result);
		void PublishPillarMarker(float x, float y);
		float radiansToDegrees(float radians) const;
		void SetupPillarMarker();
		bool StopHusky(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);

	private:
		sensor_msgs::LaserScan m_CurrentScan;
		visualization_msgs::Marker m_PillarMarker;

		ros::NodeHandle m_NodeHandle;

		ros::Publisher m_VelocityPublisher;
		ros::Publisher m_PillarMarkerPublisher;

		ros::ServiceServer m_StopHuskyServer;
		ros::ServiceClient m_StopHuskyClient;

		float m_ProcessVarTheta;
		float m_ControlGain;
		float m_ThrustSpeed;
		bool m_IsStopped;
};




#endif /* SRC_HUSKYHIGHLEVELCONTROLLER_HPP_ */
