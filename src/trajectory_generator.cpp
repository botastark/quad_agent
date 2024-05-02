#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <cmath>

// Function to generate a circular trajectory
geometry_msgs::PoseStamped generateCircularTrajectory(double radius, double angular_speed) {
    geometry_msgs::PoseStamped pose;

    // Get current time
    double time = ros::Time::now().toSec();

    // Calculate angle based on time and angular speed
    double angle = angular_speed * time;

    // Generate circular trajectory
    pose.pose.position.x = radius * cos(angle);
    pose.pose.position.y = radius * sin(angle);
    pose.pose.position.z = 2.0; // Set constant altitude

    // Set orientation (for simplicity, assume no rotation)
    pose.pose.orientation.w = 1.0;

    // Set timestamp
    pose.header.stamp = ros::Time::now();

    return pose;
}

// Function to generate a helix trajectory to ascend to a desired altitude
geometry_msgs::PoseStamped generateHelixTrajectory(double radius, double angular_speed, double desired_altitude, double start_time) {
    geometry_msgs::PoseStamped pose;

    // Get current time
    double time = ros::Time::now().toSec() - start_time;

    // Calculate angle based on time and angular speed
    double angle = angular_speed * time;

    // Generate helix trajectory
    pose.pose.position.x = radius * cos(angle);
    pose.pose.position.y = radius * sin(angle);

    // Calculate altitude based on time (ascend at a rate of 1 meter per second)
    // double current_altitude = time + 2.0; // Initial altitude + ascend rate
    double current_altitude = 0.1 * time; // Altitude increases linearly with time

    // Check if current altitude has reached desired altitude
    if (current_altitude > desired_altitude) {
        current_altitude = desired_altitude; // Limit altitude to desired altitude
    }

    pose.pose.position.z = current_altitude;

    // Set orientation (for simplicity, assume no rotation)
    pose.pose.orientation.w = 1.0;

    // Set timestamp
    pose.header.stamp = ros::Time::now();

    return pose;
}