//
// Created by asad on 09.09.19.
//

#ifndef HUSKY_HIGHLEVEL_CONTROLLER_HUSKYPROXIMITYSENSOR_H
#define HUSKY_HIGHLEVEL_CONTROLLER_HUSKYPROXIMITYSENSOR_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class HuskyProximitySensor
{
public:
    HuskyProximitySensor(const ros::NodeHandle& nodeHandle);
    ~HuskyProximitySensor();

private:
    void LaserScan(const sensor_msgs::LaserScan& laserScan);

private:
    ros::NodeHandle m_NodeHandle;
    ros::Subscriber m_Subscriber;
    ros::ServiceClient m_ServiceClient;
    float m_MinStoppingDistance;
};

#endif //HUSKY_HIGHLEVEL_CONTROLLER_HUSKYPROXIMITYSENSOR_H
