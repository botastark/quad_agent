#include "util.cpp"

std_msgs::Bool reached_target;
geometry_msgs::Vector3 current_target_global;
// std::ofstream log_file_cher;
Logger *logger;  // Pointer to Logger instance

double overall_tolerance = 0.20;  // Default value, will be updated from file
double xy_tolerance = 0.11;       // Default value, will be updated from file
double h_tolerance = 0.2;         // Default value, will be updated from file

// Callback functions
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    current_gps.pose.position.altitude =
        ellipsoid_height_to_amsl(msg->latitude, msg->longitude, msg->altitude);
    current_gps.header.stamp = msg->header.stamp;
    current_gps.pose.position.latitude = msg->latitude;
    current_gps.pose.position.longitude = msg->longitude;
}

// void stateCallback(const mavros_msgs::State::ConstPtr &msg) {
//     current_state = *msg;
// }

void targetCallback(const geographic_msgs::GeoPoseStamped::ConstPtr &msg) {
    geographic_msgs::GeoPoseStamped waypoint = *msg;
    current_target_global.x =
        waypoint.pose.position.latitude;  // Update latitude
    current_target_global.y =
        waypoint.pose.position.longitude;  // Update longitude
    current_target_global.z =
        waypoint.pose.position.altitude;  // Update altitude
}

std_msgs::Bool missionComplete() {
    double vert_dist =
        haversine(current_target_global.x, current_target_global.y,
                  current_gps.pose.position.latitude,
                  current_gps.pose.position.longitude);
    double hori_dist =
        current_gps.pose.position.altitude - current_target_global.z;
    double dist = sqrt(vert_dist * vert_dist + hori_dist * hori_dist);

    // log_file_cher << "h: " << current_gps.pose.position.altitude
    //               << " | " << current_target_global.z << std::endl;

    // log_file_cher << "lat: " << current_gps.pose.position.latitude
    //               << " | " << current_target_global.x << std::endl;

    // log_file_cher << "lon: " << current_gps.pose.position.longitude
    //               << " | " << current_target_global.y << std::endl;

    // log_file_cher << "Total: " << dist << " gps: " << vert_dist
    //               << " h: " << hori_dist << std::endl;
    logger->logMessage("h: " + std::to_string(current_gps.pose.position.altitude) +
                       " | " + std::to_string(current_target_global.z));
    logger->logMessage("lat: " + std::to_string(current_gps.pose.position.latitude) +
                       " | " + std::to_string(current_target_global.x));
    logger->logMessage("lon: " + std::to_string(current_gps.pose.position.longitude) +
                       " | " + std::to_string(current_target_global.y));
    logger->logMessage("Total: " + std::to_string(dist) + " gps: " + std::to_string(vert_dist) +
                       " h: " + std::to_string(hori_dist));

    if (dist < overall_tolerance && vert_dist < xy_tolerance && hori_dist < h_tolerance && dist != 0) {
        reached_target.data = true;
        ROS_INFO_ONCE("Reached waypoint!");
        // log_file_cher << "reached" << std::endl;
        logger->logMessage("reached");
    } else {
        reached_target.data = false;
    }

    return reached_target;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "missionchecker_node");
    ros::NodeHandle nh_;

    // Create or get the common log folder
    std::string log_folder;

    log_folder = createLogFolder();

    // ROS_INFO(log_folder);
    // Initialize the logger
    Logger mission_logger(log_folder, "mission_checker");
    logger = &mission_logger;  // Assign to global pointer

    // Example usage of logger
    logger->logMessage("Logger initialized.");

    // ros::Subscriber state_sub = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, stateCallback);
    ros::Subscriber gps_sub = nh_.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 10, gpsCallback);
    ros::Publisher reached_target_pub = nh_.advertise<std_msgs::Bool>("reached_target", 10);
    ros::Subscriber target_pos_sub = nh_.subscribe<geographic_msgs::GeoPoseStamped>("mavros/setpoint_position/global", 10, targetCallback);
    ros::Rate rate(20.0);

    // Open a log file for writing
    // log_file_cher.open("/home/bota/catkin_ws_rm/src/quad_agent/logs/mission_log.txt");

    // Check if the log file is open
    // if (!log_file_cher.is_open()) {
    //     ROS_ERROR("Failed to open log file.");
    //     return 1;
    // }
    // Read tolerances from file
    std::ifstream tol_file("/home/bota/catkin_ws_rm/src/quad_agent/path/tolerances.txt");
    if (tol_file.is_open()) {
        tol_file >> overall_tolerance >> xy_tolerance >> h_tolerance;
        tol_file.close();
    } else {
        ROS_WARN("Failed to open tolerances file. Using default values.");
    }
    // Construct log message using std::string
    std::string log_msg = "Tolerances used: overall_tolerance=" +
                          std::to_string(overall_tolerance) +
                          ", xy_tolerance=" + std::to_string(xy_tolerance) +
                          ", h_tolerance=" + std::to_string(h_tolerance);

    ROS_INFO_STREAM(log_msg);
    // Write message to log file
    // log_file_cher << log_msg << std::endl;
    // log_file_cher << "gps | target" << std::endl;
    logger->logMessage(log_msg);
    logger->logMessage("gps | target");

    while (ros::ok()) {
        reached_target_pub.publish(missionComplete());
        ros::spinOnce();
        rate.sleep();
    }

    // log_file_cher.close();
    return 0;
}
