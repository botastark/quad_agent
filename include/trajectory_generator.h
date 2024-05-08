#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <ros/ros.h>
#include <cmath>

// Function prototype for generating a circular trajectory
geometry_msgs::PoseStamped generateCircularTrajectory(double radius, double angular_speed);
geometry_msgs::PoseStamped generateHelixTrajectory(double radius, double angular_speed, double desired_altitude, double start_time);
mavros_msgs::PositionTarget gen_pos_msgs(const geometry_msgs::PoseStamped& new_pose, 
                                const geometry_msgs::Vector3& new_velocity);
mavros_msgs::PositionTarget calculateSmoothTrajectory(const geometry_msgs::PoseStamped& initial_pose,
                               const geometry_msgs::PoseStamped& final_pose,
                               const geometry_msgs::Vector3& initial_velocity,
                               const geometry_msgs::Vector3& final_velocity,
                               double duration,
                               double start_time);

#endif // TRAJECTORY_GENERATOR_H
