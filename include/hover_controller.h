#ifndef HOVER_CONTROLLER_H
#define HOVER_CONTROLLER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/Thrust.h>

// Function prototypes
void stateCallback(const mavros_msgs::State::ConstPtr& msg);
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
void controlMotors(double error);

// Global variable declarations
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
mavros_msgs::Thrust thrust;
ros::Publisher thrust_pub;
#endif /* HOVER_CONTROLLER_H */
