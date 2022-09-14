
#include "ros/node_handle.h"
#include <ros/ros.h>
#include <iostream>
#include <sr_hand/hand_commander.hpp>
// #include <sr_robot_msgs.h>
#include <sr_robot_msgs/joint.h>
int main (int argc, char **argv)
{
    // init node
    ros::init(argc, argv, "sr_test");

    ros::NodeHandle nh;

    // load hand controller 
    auto hc = shadowrobot::HandCommander();
    auto a = hc.get_all_joints();

    std::vector<sr_robot_msgs::joint> joints;
    for (auto& j : joints)
        j.joint_target = 0.0;

    b.joint_target = 0.1;

    ros::Rate rate(1); // ROS Rate at 5Hz
    
    while ( ros::ok() )
	{
        ROS_ERROR(a[0].c_str());
        ROS_ERROR("Hello");
        rate.sleep();
    }
}

/*
stuff to remember:
 + #include "sr_tactile_sensors/sr_virtual_tactile_sensor.hpp"

*/