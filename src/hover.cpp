#include "hover_controller.h"
#include "trajectory_generator.h"

// Global variable to store LiDAR data
// double lidar_distance = 0.0;

// // Callback fn for state from mavros
void stateCallback(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

// Callback fn for pose from mavros
// void poseCallback(const geometry_msgs::PoseStamped::C::onstPtr& msg){
//     current_pose = *msg;
// }
// Callback function for LiDAR data
// void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
//     // Let's take average of range (for now we know that there are five rays)
//     lidar_distance = (msg->ranges[0]+msg->ranges[1]+msg->ranges[2]+msg->ranges[3]+msg->ranges[4])/5;
// }

int main(int argc, char **argv) {
    ros::init(argc, argv, "hover_node");
    ros::NodeHandle nh;

    // ros::Subscriber sub = nh.subscribe("/agent/laser/scan", 1000, lidarCallback);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, stateCallback);
    // ros::Subscriber local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
    //         ("mavros/local_position/pose", 10, poseCallback);
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

    geometry_msgs::PoseStamped poseA;
    poseA.pose.position.x = 0;
    poseA.pose.position.y = 0;
    poseA.pose.position.z = 0;

    // send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(poseA);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while (ros::ok()) {
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

        // Generate circular trajectory (radius = 1.0, angular_speed = 0.1)
        geometry_msgs::PoseStamped new_pose = generateCircularTrajectory(1.0, 0.1);

        // to set a pose to a drone
        local_pos_pub.publish(new_pose);

        // Spin once to handle callbacks
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
