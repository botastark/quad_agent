#include "util.cpp"
double altitude_hrlv = 0.0;
bool reached_target = false;
bool current_gps_received = false;

#include <sensor_msgs/Range.h>

void altitudeCallback(const sensor_msgs::Range::ConstPtr &msg) {
    altitude_hrlv = msg->range;
}

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    current_gps_received = true;
    current_gps.pose.position.altitude =
        ellipsoid_height_to_amsl(msg->latitude, msg->longitude, msg->altitude);
    current_gps.header.stamp = msg->header.stamp;
    current_gps.pose.position.latitude = msg->latitude;
    current_gps.pose.position.longitude = msg->longitude;
}

void stateCallback(const mavros_msgs::State::ConstPtr &msg) {
    current_state = *msg;
}

void reachedTargetCallback(const std_msgs::Bool::ConstPtr &msg) {
    reached_target = msg->data;
}

// TODO: Function to take a picture
void takePicture(ros::Publisher &take_picture_pub) {
    std_msgs::Bool msg;
    msg.data = true;
    take_picture_pub.publish(msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mission_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, stateCallback);
    ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 10, gpsCallback);
    ros::Subscriber mission_complete_sub = nh.subscribe<std_msgs::Bool>("reached_target", 10, reachedTargetCallback);

    ros::Subscriber sub = nh.subscribe<sensor_msgs::Range>("/mavros/distance_sensor/hrlv_ez4_pub", 10, altitudeCallback);

    ros::Publisher global_pos_pub = nh.advertise<geographic_msgs::GeoPoseStamped>("mavros/setpoint_position/global", 10);
    ros::Publisher take_picture_pub = nh.advertise<std_msgs::Bool>("take_picture", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::Rate rate(20.0);

    // Wait for FCU connection
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    // wait for position information
    while (ros::ok() && !current_gps_received) {
        ROS_INFO_ONCE("Waiting for GPS signal...");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("GPS position received");

    geographic_msgs::GeoPoseStamped goal_position, home_position;
    home_position = create_pose(current_gps.pose.position.latitude,
                                current_gps.pose.position.longitude,
                                current_gps.pose.position.altitude);
    ROS_INFO("HOME POSITION");
    ROS_INFO_STREAM(home_position);

    std::string filename = "/home/bota/catkin_ws_rm/src/quad_agent/path/waypoints.txt";  // File containing waypoints

    std::vector<GPSPosition> waypoints = readWaypointsFromFile(filename, current_gps.pose.position.altitude);
    double temp_home_alt = home_position.pose.position.altitude;

    // send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i) {
        home_position.header.stamp = ros::Time::now();
        home_position.pose.position.altitude = temp_home_alt + 2.5;
        global_pos_pub.publish(home_position);
        ros::spinOnce();
        rate.sleep();
    }
    // home_position.pose.position.altitude = temp_home_alt;

    setMode(set_mode_client, "OFFBOARD");
    ROS_INFO("Drone taking off");

    // Wait for offboard
    while (ros::ok() && current_state.mode != "OFFBOARD") {
        ros::spinOnce();
        rate.sleep();
    }
    armDrone(arming_client);

    // Navigate to each waypoint
    for (const auto &waypoint : waypoints) {
        double goal_alt = 92 + waypoint.altitude - altitude_hrlv;
        ROS_INFO_STREAM("Navigating to waypoint: "
                        << std::to_string(waypoint.latitude) << ", "
                        << std::to_string(waypoint.longitude) << ", "
                        << std::to_string(waypoint.altitude) << ", " << current_gps.pose.position.altitude << "goal alt" << goal_alt);
        goal_position = create_pose(waypoint.latitude, waypoint.longitude, goal_alt);
        global_pos_pub.publish(goal_position);
        // Wait for waypoint reached
        while (ros::ok() && !reached_target) {
            // double goal_alt = current_gps.pose.position.altitude + waypoint.altitude - altitude_hrlv;
            // goal_position = create_pose(waypoint.latitude, waypoint.longitude, goal_alt);
            // ROS_INFO_STREAM("Navigating to waypoint: "
            //                 << std::to_string(waypoint.latitude) << ", "
            //                 << std::to_string(waypoint.longitude) << ", "
            //                 << std::to_string(waypoint.altitude) << ", " << current_gps.pose.position.altitude << "goal alt" << goal_alt);
            // ROS_INFO_STREAM(goal_position);
            // ROS_INFO_STREAM("gps: "
            //                 << current_gps.pose.position.altitude << ", waypoint.altitude: " << waypoint.altitude << ", dist sensor" << altitude_hrlv);
            global_pos_pub.publish(goal_position);
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO("reached waypoint");

        // Hover for 1 second to stabilize the drone
        ros::Time start_time = ros::Time::now();
        while (ros::ok() && (ros::Time::now() - start_time).toSec() < 1.0) {
            global_pos_pub.publish(goal_position);
            ros::spinOnce();
            rate.sleep();
        }

        // Take picture at waypoint
        takePicture(take_picture_pub);
        ROS_INFO("Taking picture");

        start_time = ros::Time::now();
        while (ros::ok() && (ros::Time::now() - start_time).toSec() < 1.0) {
            global_pos_pub.publish(goal_position);
            ros::spinOnce();
            rate.sleep();
        }
    }

    // Return to home
    while (ros::ok() && !reached_target) {
        home_position = create_pose(
            home_position.pose.position.latitude, home_position.pose.position.longitude, home_position.pose.position.altitude);
        global_pos_pub.publish(home_position);
        ROS_INFO("Returning to home");
        ros::spinOnce();
        rate.sleep();
    }

    // Wait for landing
    setMode(set_mode_client, "AUTO.LAND");
    ROS_INFO("Drone landing");

    while (ros::ok() && current_state.mode != "AUTO.LAND") {
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Mission complete");

    return 0;
}
