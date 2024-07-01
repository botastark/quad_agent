#include "util.cpp"
// double altitude_hrlv = 0.0;
bool reached_target = false;
bool current_gps_received = false;

#include <sensor_msgs/Range.h>

// void altitudeCallback(const sensor_msgs::Range::ConstPtr &msg) {
//     altitude_hrlv = msg->range;
// }

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
    std::string log_folder = createLogFolder();

    {
        Logger logger(log_folder, "mission_log");
        // initLogFile();

        ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, stateCallback);
        ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 10, gpsCallback);
        ros::Subscriber mission_complete_sub = nh.subscribe<std_msgs::Bool>("reached_target", 10, reachedTargetCallback);
        //
        // ros::Subscriber sub = nh.subscribe<sensor_msgs::Range>("/mavros/distance_sensor/hrlv_ez4_pub", 10, altitudeCallback);

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

        ROS_INFO("FCU connected");
        logger.logMessage("FCU connected!");

        // wait for position information
        while (ros::ok() && !current_gps_received) {
            ROS_INFO_ONCE("Waiting for GPS signal...");
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO("GPS position received");
        logger.logMessage("GPS position received");

        geographic_msgs::GeoPoseStamped goal_position, home_position;
        home_position = create_pose(current_gps.pose.position.latitude,
                                    current_gps.pose.position.longitude,
                                    current_gps.pose.position.altitude);
        ROS_INFO("HOME POSITION");
        ROS_INFO_STREAM(home_position);

        logger.logMessage("HOME POSITION: lat=" + std::to_string(home_position.pose.position.latitude) +
                          ", lon=" + std::to_string(home_position.pose.position.longitude) +
                          ", alt=" + std::to_string(home_position.pose.position.altitude));

        std::string filename = "/home/bota/catkin_ws_rm/src/quad_agent/path/waypoints.txt";  // File containing waypoints

        std::vector<GPSPosition> waypoints = readWaypointsFromFile(filename, current_gps.pose.position.altitude);
        if (waypoints.empty()) {
            logger.logMessage("Couldn't read waypoints file");
        }
        double temp_home_alt = home_position.pose.position.altitude;

        // send a few setpoints before starting
        for (int i = 100; ros::ok() && i > 0; --i) {
            home_position.header.stamp = ros::Time::now();
            home_position.pose.position.altitude = temp_home_alt + 2.5;
            global_pos_pub.publish(home_position);
            ros::spinOnce();
            rate.sleep();
        }
        logger.logMessage("Sending a few point before starting");

        setMode(set_mode_client, "OFFBOARD");
        // Waiting for OFFBOARD
        // Wait for offboard (setting to offboard is done via RC/QGC)
        while (ros::ok() && current_state.mode != "OFFBOARD") {
            home_position.header.stamp = ros::Time::now();
            global_pos_pub.publish(home_position);
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO("Drone is in OFFBOARD mode.");
        logger.logMessage("Drone is in OFFBOARD mode.");

        armDrone(arming_client);
        while (ros::ok() && !current_state.armed) {
            home_position.header.stamp = ros::Time::now();
            global_pos_pub.publish(home_position);
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO("Drone is armed.");
        logger.logMessage("Drone is armed.");
        ros::Time start_time;

        // Navigate to each waypoint
        for (const auto &waypoint : waypoints) {
            std::string waypoint_log = "Navigating to waypoint: " + std::to_string(waypoint.latitude) + ", " + std::to_string(waypoint.longitude) + ", " + std::to_string(waypoint.altitude + temp_home_alt);
            logger.logMessage(waypoint_log);
            ROS_INFO_STREAM(waypoint_log);

            goal_position = create_pose(waypoint.latitude, waypoint.longitude, waypoint.altitude + temp_home_alt);
            global_pos_pub.publish(goal_position);
            // Wait for waypoint reached
            while (ros::ok() && !reached_target) {
                goal_position.header.stamp = ros::Time::now();
                global_pos_pub.publish(goal_position);
                ros::spinOnce();
                rate.sleep();
            }
            ROS_INFO("reached waypoint");
            logger.logMessage("reached waypoint");

            // Hover for 1 second to stabilize the drone
            ros::Time start_time = ros::Time::now();
            while (ros::ok() && (ros::Time::now() - start_time).toSec() < 1.0) {
                goal_position.header.stamp = ros::Time::now();
                global_pos_pub.publish(goal_position);
                ros::spinOnce();
                rate.sleep();
            }

            // Take picture at waypoint
            takePicture(take_picture_pub);
            ROS_INFO("Taking picture");
            logger.logMessage("Taking picture");

            start_time = ros::Time::now();
            while (ros::ok() && (ros::Time::now() - start_time).toSec() < 1.0) {
                goal_position.header.stamp = ros::Time::now();
                global_pos_pub.publish(goal_position);
                ros::spinOnce();
                rate.sleep();
            }
        }

        // Return to home
        home_position.header.stamp = ros::Time::now();
        global_pos_pub.publish(home_position);
        while (ros::ok() && !reached_target) {
            home_position.header.stamp = ros::Time::now();
            global_pos_pub.publish(home_position);
            ROS_INFO_ONCE("Returning to home");
            logger.logMessage("Returning to home");
            ros::spinOnce();
            rate.sleep();
        }

        // Wait for landing

        setMode(set_mode_client, "AUTO.LAND");

        ROS_INFO("Drone start landing");
        logger.logMessage("Drone start landing");

        ros::Time land_start = ros::Time::now();
        while (ros::ok() && current_state.mode != "AUTO.LAND") {
            setMode(set_mode_client, "AUTO.LAND");
            ROS_INFO_ONCE("Drone setting to land");
            logger.logMessage("Drone keep setting to land");
            ros::spinOnce();
            rate.sleep();
        }
        home_position.pose.position.altitude = temp_home_alt;
        // Wait for 15 seconds after landing (regardless of landing confirmation)
        while (ros::ok() && !reached_target) {
            goal_position.header.stamp = ros::Time::now();
            global_pos_pub.publish(home_position);
            ROS_INFO_ONCE("Drone landing");
            logger.logMessage("Drone keep setting to land");
            ros::spinOnce();
            rate.sleep();
        }
        if (reached_target) {
            ROS_INFO("Drone has landed");
            logger.logMessage("Drone has landed");
        }

        ROS_INFO("Mission complete");
        logger.logMessage("Mission complete");
    }
    // closeLogFile();
    return 0;
}
