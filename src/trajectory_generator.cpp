#include "trajectory_generator.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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

    geometry_msgs::Vector3 direction;
    direction.x = pose.pose.position.x;
    direction.y = pose.pose.position.y;
    direction.z = 0;
    
    double pitch = atan2(direction.y, direction.x); //in radians


    // Set orientation (for simplicity, assume no rotation)
    tf2::Quaternion quad_tf;
    
    quad_tf.setRPY( 0.0,  pitch,  0.0);
    geometry_msgs::Quaternion orientation;
    tf2::convert(quad_tf,orientation);
    ROS_INFO_STREAM("pitch "<<pitch<< " quat "<<orientation);

    pose.pose.orientation = orientation;
    // euler2quaternion

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


// Function to calculate distance between two poses
double distance(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2) {
    double dx = pose1.pose.position.x - pose2.pose.position.x;
    double dy = pose1.pose.position.y - pose2.pose.position.y;
    double dz = pose1.pose.position.z - pose2.pose.position.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}


mavros_msgs::PositionTarget  gen_pos_msgs(const geometry_msgs::PoseStamped& new_pose, 
                                        const geometry_msgs::Vector3& new_velocity){

    mavros_msgs::PositionTarget pos_target_;

    pos_target_.header.stamp = ros::Time::now();
    pos_target_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    pos_target_.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
                            mavros_msgs::PositionTarget::FORCE | mavros_msgs::PositionTarget::IGNORE_YAW | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    pos_target_.position.x = new_pose.pose.position.x;
    pos_target_.position.y = new_pose.pose.position.y;
    pos_target_.position.z = new_pose.pose.position.z;

    pos_target_.velocity.x = new_velocity.x;
    pos_target_.velocity.y = new_velocity.y;
    pos_target_.velocity.z = new_velocity.z;

    return pos_target_;
}

// Function to calculate a smooth trajectory between initial and final positions with velocities
mavros_msgs::PositionTarget calculateSmoothTrajectory(const geometry_msgs::PoseStamped& initial_pose,
                               const geometry_msgs::PoseStamped& final_pose,
                               const geometry_msgs::Vector3& initial_velocity,
                               const geometry_msgs::Vector3& final_velocity,
                               double duration,
                               double start_time) {

    geometry_msgs::PoseStamped new_pose;
    geometry_msgs::Vector3 new_velocity;

    double tau = (ros::Time::now().toSec() - start_time)/duration;
    if (start_time==0.0){
        return gen_pos_msgs(initial_pose, initial_velocity);
    }
    if (tau>=1){
        return gen_pos_msgs(final_pose,final_velocity);
    }
    // ROS_INFO_STREAM("tau: "<<tau);
        
    // Calculate coefficients for cubic spline interpolation
    double dqx = final_pose.pose.position.x-initial_pose.pose.position.x;
    double dqy = final_pose.pose.position.y-initial_pose.pose.position.y;
    double dqz = final_pose.pose.position.z-initial_pose.pose.position.z;

    double ax = -2*dqx + duration*(initial_velocity.x+final_velocity.x);
    double ay = -2*dqy + duration*(initial_velocity.y+final_velocity.y);
    double az = -2*dqz + duration*(initial_velocity.z+final_velocity.z);

    double bx = 3*dqx - duration*(2*initial_velocity.x+final_velocity.x);
    double by = 3*dqy - duration*(2*initial_velocity.y+final_velocity.y);
    double bz = 3*dqz - duration*(2*initial_velocity.z+final_velocity.z);

    double cx = duration*initial_velocity.x;
    double cy = duration*initial_velocity.y;
    double cz = duration*initial_velocity.z;

    // Calculate position using cubic spline interpolation
    new_pose.pose.position.x = initial_pose.pose.position.x + cx * tau + bx * pow(tau, 2) + ax * pow(tau, 3);
    new_pose.pose.position.y = initial_pose.pose.position.y + cy * tau + by * pow(tau, 2) + ay * pow(tau, 3);
    new_pose.pose.position.z = initial_pose.pose.position.z + cz * tau + bz * pow(tau, 2) + az * pow(tau, 3);

    // Calculate velocity using first derivative of cubic spline interpolation
    new_velocity.x = 1/duration * (cx + 2 * bx * tau + 3 * ax * pow(tau, 2));
    new_velocity.y = 1/duration * (cy + 2 * by * tau + 3 * ay * pow(tau, 2));
    new_velocity.z = 1/duration * (cz + 2 * bz * tau + 3 * az * pow(tau, 2));

    return gen_pos_msgs(new_pose, new_velocity);
}                                   
