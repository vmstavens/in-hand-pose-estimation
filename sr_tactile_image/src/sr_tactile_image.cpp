#include <cstddef>
#include <ros/ros.h>
#include <iostream>
#include "control_msgs/JointControllerState.h"
#include "ros/duration.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/service.h"
#include "ros/subscriber.h"
#include "sensor_msgs/JointState.h"
#include <sr_hand/sr_subscriber.h>
#include "sr_hand/hand/sr_articulated_robot.h"
#include "sr_hand/hand/virtual_shadowhand.h"
#include "sr_hand/hand_commander.hpp"
#include "sr_robot_msgs/joints_data.h"
#include <sr_utilities/sr_hand_finder.hpp>

void callback_command()
{
    ROS_WARN_STREAM("I'm in a callback....");
}


int main (int argc, char **argv)
{
    
	// std::vector<sr_robot_msgs::joint> home;

    const std::string NODE_NAME = "sr_tactile_image";

    // init node
    ros::init(argc, argv, NODE_NAME);

    ros::NodeHandle nh;

    /*
        • SRsubscriber/SRpublisher should not be used manually as they act from within the hand
        • HandCommander needs a string, but i don't know which one? I have tried rostopic list -v [nodes] but no result
        • SrHandFinder with no argument does not link, the ones shown below are tested
            shadow_robot::SrHandFinder hf; 
            shadow_robot::SrHandFinder hf = shadow_robot::SrHandFinder();
        •
        # Plan
        • Run the simulation
        • echo services and look for X +"/controller_manager/list_controllers" and locate X
        • ros::service::waitForService("/sh_rh_wrj2_position_controller/set_gains",WAIT_TIME); // improve by waiting for service to be available or topic?
    */

    const double WAIT_TIME = 3.0; // load from ros param server instead and put in config file

    ros::Rate rate(1.0/WAIT_TIME); // ROS Rate Hz
    rate.sleep();

    shadowrobot::HandCommander hc = shadowrobot::HandCommander("/");
    auto joint_names = hc.get_all_joints();
    ROS_ERROR_STREAM("THE NUMBER OF JOINT NAMES FOUND : " << joint_names.size() << " ################################################");
    auto joint_of_interest = joint_names[0];
    // 1 | get the angle
    ros::Subscriber sub = nh.subscribe<control_msgs::JointControllerState>("/sh_rh_ffj0_position_controller/state",1,callback_command);
    // ros::Subscriber sub = nh.subscribe<sensor_msgs::JointState>(joint_of_interest,1,callback_command);
    // └──╼ rostopic pub /sh_rh_ffj0_position_controller/command std_msgs/Float64 "data: 1.0" 
    // /sh_rh_ffj0_position_controller/state [control_msgs/JointControllerState] 1 publisher

    // ros::Subscriber sub = nh.subscribe<>(joint_of_interest,1,callback_command);

    while ( ros::ok() )
	{
        ROS_ERROR_STREAM(joint_names[0]);
        ROS_WARN_STREAM("Hello");
        rate.sleep();
    }
}

/*
stuff to remember:
 + #include "sr_tactile_sensors/sr_virtual_tactile_sensor.hpp"
*/