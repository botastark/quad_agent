#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

// Global variable to store LiDAR data
double lidar_distance = 0.0;
double prev = 0.0;
mavros_msgs::State current_state;

// Callback fn for state from mavros
void stateCallback(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

// Callback function for LiDAR data
void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {

    // Assuming LiDAR data is in the form of a LaserScan message
    // Let's take average of range (for now we know that there are five rays)
    lidar_distance = (msg->ranges[0]+msg->ranges[1]+msg->ranges[2]+msg->ranges[3]+msg->ranges[4])/5;
}

// Function to control drone motors
void controlMotors(double error) {
    // Proportional control: Adjust motor speeds based on error
    double Kp = 0.1; // Proportional gain
    double thrust = Kp * error;

    // Apply the thrust to the motors
    // Example: adjust motor speeds accordingly
    // motor_speed = base_speed + thrust;
    // Adjust each motor accordingly
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "hover_node");

    ros::NodeHandle nh;
    // Don't exit the program.
    // ros::spin();


    // Wait for a minute for Gazebo to launch 
    ros::Duration(60).sleep();


    ros::Subscriber sub = nh.subscribe("/agent/laser/scan", 1000, lidarCallback);
    prev = lidar_distance;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, stateCallback);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }


    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;


    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    while (ros::ok()) {
        // Compute error: double error = desired_height - lidar_distance;

        // Control motors based on error
        // controlMotors(error);

        // Output current height for monitoring (optional)
        if (prev!=lidar_distance){
            ROS_INFO("Current Height: %.2f meters", lidar_distance);
            prev = lidar_distance;
        }

        // make sure that mode is set to offboard and drone is armed within 5sec btw requests
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        // to set a pose to a drone
        local_pos_pub.publish(pose);

        // Spin once to handle callbacks
        ros::spinOnce();
        // sleep(0.1); // 100 milliseconds
    }

    return 0;
}
