/*
 * HuskyHighLevelController.hpp
 *
 *  Created on: Sep 1, 2019
 *      Author: Asad Ahmed
 */

#ifndef SRC_HUSKYHIGHLEVELCONTROLLER_HPP_
#define SRC_HUSKYHIGHLEVELCONTROLLER_HPP_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class HuskyHighLevelController
{
	public:
		HuskyHighLevelController(const ros::NodeHandle& handle);
		~HuskyHighLevelController();

		void ReadScan(const sensor_msgs::LaserScan& scan);

	private:
		ros::NodeHandle m_NodeHandle;
};




#endif /* SRC_HUSKYHIGHLEVELCONTROLLER_HPP_ */
