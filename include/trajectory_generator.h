#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <geometry_msgs/PoseStamped.h>

// Function prototype for generating a circular trajectory
geometry_msgs::PoseStamped generateCircularTrajectory(double radius, double angular_speed);
geometry_msgs::PoseStamped generateHelixTrajectory(double radius, double angular_speed, double desired_altitude, double start_time);
#endif // TRAJECTORY_GENERATOR_H
