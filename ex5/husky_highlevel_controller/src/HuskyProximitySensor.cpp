// HuskyProximitySensor.cpp

#include "HuskyProximitySensor.h"

#include <std_srvs/SetBool.h>

HuskyProximitySensor::HuskyProximitySensor(const ros::NodeHandle &nodeHandle) : m_NodeHandle(nodeHandle), m_MinStoppingDistance(1.0)
{
    // get params
    int queueSize = 1;
    nodeHandle.getParam("queue_size", queueSize);
    nodeHandle.getParam("min_distance_to_stop", m_MinStoppingDistance);

    // subscribe to laser scan
    m_Subscriber = m_NodeHandle.subscribe("/scan", queueSize, &HuskyProximitySensor::LaserScan, this);

    // setup stopping client
    m_ServiceClient = m_NodeHandle.serviceClient<std_srvs::SetBool>("/stop_husky");
}

HuskyProximitySensor::~HuskyProximitySensor() {}

// Laser scan callback
void  HuskyProximitySensor::LaserScan(const sensor_msgs::LaserScan& laserScan)
{
    // detect short distance and stop
    auto iter = std::min_element(laserScan.ranges.begin(), laserScan.ranges.end());

    // for dealing with noise and erratic sensor readings
    const float laserRangeTolerance = 2.0;

    if (iter != laserScan.ranges.end())
    {
        if ((*iter) <= m_MinStoppingDistance && (*iter) >= laserRangeTolerance)
        {
            // stop husky by calling the stop service
            std_srvs::SetBool service;
            service.request.data = true;

            if (m_ServiceClient.call(service)) {
                ROS_INFO("Husky Proximity Sensor detected obstacle at distance: %f", *iter);
            }
            else {
                ROS_ERROR("Failed to stop Husky");
            }
        }
    }
}