#include "hover_controller.h"
#include "trajectory_generator.h"
#include <mavros_msgs/PositionTarget.h>

// Global variable to store LiDAR data
// double lidar_distance = 0.0;

// // Callback fn for state from mavros
void stateCallback(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

// void commandCallback(const bool& msg){
//     start_mission = *msg;
// }

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
    // ros::Subscriber external_commands = nh.subscribe<bool>
            // ("missioncommand", 10, poseCallback);

    // ros::Subscriber local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
    //         ("mavros/local_position/pose", 10, poseCallback);

    // ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
    //         ("mavros/setpoint_position/local", 10);

    ros::Publisher pos_target_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/local", 10);

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

    // geometry_msgs::PoseStamped poseA;
    // poseA.pose.position.x = 0;
    // poseA.pose.position.y = 0;
    // poseA.pose.position.z = 0;


    // Define initial and final poses
    geometry_msgs::PoseStamped initial_pose;
    initial_pose.pose.position.x = 0.0;
    initial_pose.pose.position.y = 0.0;
    initial_pose.pose.position.z = 0.0;

    geometry_msgs::PoseStamped final_pose;
    final_pose.pose.position.x = 1.0;
    final_pose.pose.position.y = 1.0;
    final_pose.pose.position.z = 2.0;

    // Define initial and final velocities
    geometry_msgs::Vector3 initial_velocity;
    initial_velocity.x = 0.0;
    initial_velocity.y = 0.0;
    initial_velocity.z = 0.0;

    geometry_msgs::Vector3 final_velocity;
    final_velocity.x = 0.0;
    final_velocity.y = 0.0;
    final_velocity.z = 0.0;

    mavros_msgs::PositionTarget pos_target;

    // Define duration of the trajectory
    double duration = 10.0; // 10 seconds
    pos_target = gen_pos_msgs(initial_pose, initial_velocity);

    // send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        // local_pos_pub.publish(poseA);
        pos_target_pub.publish(pos_target);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    // Calculate start offset
    double start_time = 0.0; // Start immediately


    while( ros::ok()  ){
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
                    start_time = ros::Time::now().toSec();
                }
                last_request = ros::Time::now();
            }
        }
            // geometry_msgs::PoseStamped new_pose = generateCircularTrajectory(2.0, 1);
            // geometry_msgs::PoseStamped new_pose = generateHelixTrajectory(1.0, 0.1, 3.0, start_time);
            // local_pos_pub.publish(new_pose);
        pos_target = calculateSmoothTrajectory(initial_pose, final_pose, initial_velocity, final_velocity, duration, start_time);
        pos_target_pub.publish(pos_target);
        // double tau = (ros::Time::now().toSec() - start_time)/duration;
        // if (tau<1.1){
        //     ROS_INFO_STREAM("now: "<< ros::Time::now().toSec() - start_time <<" tau: " << tau);
        //     ROS_INFO_STREAM("curr target pose "<< pos_target);
        // }

        
        
            
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
