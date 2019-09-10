// husky_proximty_detection_node.cpp

#include <ros/ros.h>

#include "HuskyProximitySensor.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "husky_proximity_detection");
    ros::NodeHandle nodeHandle("~");

    // create proximity sensor
    HuskyProximitySensor proximitySensor { nodeHandle };

    ros::spin();

    return 0;
}

