#include <iostream>
#include "hover_controller.h"

// Global variable to store LiDAR data
double lidar_distance = 0.0;
double prev = 0.0;

// Callback fn for state from mavros
void stateCallback(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

// Callback fn for pose from mavros
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}
// Callback function for LiDAR data
void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // Let's take average of range (for now we know that there are five rays)
    lidar_distance = (msg->ranges[0]+msg->ranges[1]+msg->ranges[2]+msg->ranges[3]+msg->ranges[4])/5;
}

// Function to control drone motors
double temp_prev = 0.0;
void controlMotors(double error) {
    // Proportional control: Adjust motor speeds based on error
    double Kp = 0.1; // Proportional gain
    thrust.thrust = (float) Kp * error;
    float temp = (float) Kp * error;

    // ROS_INFO(temp);
    // if ((temp_prev-temp>0.01 &&  temp_prev>temp ) || (temp_prev-temp<0.01 &&  temp_prev<temp ) ){
    //     ROS_INFO("Current Thrust: %.2f ", temp);
    //     temp_prev=temp;
    // }
    

    // thrust_pub.publish(thrust);

    // Apply the thrust to the motors
    // Example: adjust motor speeds accordingly
    // motor_speed = base_speed + thrust;
    // Adjust each motor accordingly
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "hover_node");
    ros::NodeHandle nh;
    // Wait for a minute for Gazebo to launch 
    // ros::Duration(60).sleep();


    ros::Subscriber sub = nh.subscribe("/agent/laser/scan", 1000, lidarCallback);
    prev = lidar_distance;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, stateCallback);
    ros::Subscriber local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, poseCallback);
    
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    thrust_pub = nh.advertise<mavros_msgs::Thrust>
            ("mavros/setpoint_attitude/thrust", 10);

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

    geometry_msgs::PoseStamped poseB;
    poseB.pose.position.x = 0;
    poseB.pose.position.y = 0;
    poseB.pose.position.z = 3;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(poseB);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    while (ros::ok()) {
        // Output current height for monitoring (optional)
        // if (prev!=lidar_distance){
        //     ROS_INFO("Current Height: %.2f meters", lidar_distance);
        //     prev = lidar_distance;
        // }

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
        local_pos_pub.publish(poseB);

        // Compute error: 
        // double error = poseB.pose.position.z - lidar_distance;

        // Control motors based on error
        // controlMotors(error);


        // Spin once to handle callbacks
        ros::spinOnce();
        // sleep(0.1); // 100 milliseconds
    }

    return 0;
}
