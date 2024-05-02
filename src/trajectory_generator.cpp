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
