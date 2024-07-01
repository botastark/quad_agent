#include "mission.h"
// Need to WGS84->amsl of gps altitude
//"When controlling the FCU using global setpoints, you specify the altitude as
// meters above mean sea level (AMSL). But when sensing the global position, the
// altitude reported by ~global_position/global is specified as meters above the
// WGS-84 ellipsoid. This can lead to differences in altitude that are dozens of
// meters apart."
// https://wiki.ros.org/mavros#mavros.2FPlugins.Avoiding_Pitfalls_Related_to_Ellipsoid_Height_and_Height_Above_Mean_Sea_Level

double calc_geoid_height(double lat, double lon) { return _egm96(lat, lon); }
double amsl_to_ellipsoid_height(double lat, double lon, double amsl) {
    return amsl +
           GeographicLib::Geoid::GEOIDTOELLIPSOID * calc_geoid_height(lat, lon);
}
double ellipsoid_height_to_amsl(double lat, double lon,
                                double ellipsoid_height) {
    return ellipsoid_height +
           GeographicLib::Geoid::ELLIPSOIDTOGEOID * calc_geoid_height(lat, lon);
}

double haversine(double lat1, double lon1, double lat2, double lon2) {
    // Constants
    const double R = 6371000.0;  // Earth radius in meters
    // Convert latitude and longitude from degrees to radians
    double lat1_rad = lat1 * M_PI / 180.0;
    double lon1_rad = lon1 * M_PI / 180.0;
    double lat2_rad = lat2 * M_PI / 180.0;
    double lon2_rad = lon2 * M_PI / 180.0;
    // Haversine formula
    double dlat = (lat2 - lat1) * M_PI / 360.0;
    double dlon = (lon2 - lon1) * M_PI / 360.0;
    double a = std::sin(dlat) * std::sin(dlat) +
               std::cos(lat1_rad) * std::cos(lat2_rad) * std::sin(dlon) *
                   std::sin(dlon);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
    double distance = R * c;
    return distance;
}

// Function to read waypoints from a file and add current GPS altitude
std::vector<GPSPosition> readWaypointsFromFile(const std::string &filename, double currentGpsAltitude) {
    std::vector<GPSPosition> waypoints;
    std::ifstream infile(filename);

    if (!infile.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return waypoints;
    }

    std::string line;
    while (std::getline(infile, line)) {
        // Remove brackets and commas
        line.erase(std::remove(line.begin(), line.end(), '['), line.end());
        line.erase(std::remove(line.begin(), line.end(), ']'), line.end());
        line.erase(std::remove(line.begin(), line.end(), ','), line.end());

        // Create a stringstream to parse the line
        std::stringstream ss(line);
        double lat, lon, alt;
        ss >> lat >> lon >> alt;

        // Add current GPS altitude to the read altitude
        // alt += currentGpsAltitude;

        // Add the waypoint to the vector
        waypoints.push_back({lat, lon, alt});
    }

    infile.close();
    return waypoints;
}

// Function to create geo msgs for a GPS waypoint
geographic_msgs::GeoPoseStamped create_pose(double latitude,
                                            double longitude,
                                            double altitude) {
    geographic_msgs::GeoPoseStamped waypoint;
    waypoint.header.stamp = ros::Time::now();
    waypoint.header.frame_id = "map";  // Frame should be "map" for GPS waypoints
    waypoint.pose.position.latitude = latitude;
    waypoint.pose.position.longitude = longitude;
    waypoint.pose.position.altitude = altitude;
    return waypoint;
}

void armDrone(ros::ServiceClient &arming_client) {
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Drone armed");
    } else {
        ROS_ERROR("Failed to arm the drone");
    }
}

void setMode(ros::ServiceClient &set_mode_client, const std::string &mode) {
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = mode;
    if (set_mode_client.call(offb_set_mode) &&
        offb_set_mode.response.mode_sent) {
        ROS_INFO_STREAM("Mode set to: " << mode);
    } else {
        ROS_ERROR_STREAM("Failed to set mode: " << mode);
    }
}

// // Function to initialize the log file with a unique name based on timestamp
// void initLogFile() {
//     std::stringstream log_file_name;
//     log_file_name << "/home/bota/catkin_ws_rm/src/quad_agent/logs/mission_node_log_" << ros::Time::now() << ".txt";
//     log_file.open(log_file_name.str());
//     if (!log_file.is_open()) {
//         ROS_ERROR("Failed to open log file: %s", log_file_name.str().c_str());
//     } else {
//         ROS_INFO("Logging to file: %s", log_file_name.str().c_str());
//     }
// }

// void logMessage(const std::string &message) {
//     std::ostringstream oss;
//     oss << "[" << ros::Time::now() << "] " << message;

//     // Log to ROS
//     ROS_INFO_STREAM(oss.str());

//     // Log to file
//     if (log_file.is_open()) {
//         log_file << oss.str() << std::endl;
//         log_file.flush();  // Ensure data is written immediately
//     } else {
//         ROS_WARN("Log file is not open, message not logged to file: %s", message.c_str());
//     }
// }

// // Function to close the log file
// void closeLogFile() {
//     if (log_file.is_open()) {
//         log_file.close();
//     }
// }

class Logger {
   public:
    Logger(const std::string &folder_name, const std::string &file_name_prefix) {
        initLogFile(folder_name, file_name_prefix);
    }

    ~Logger() {
        closeLogFile();
    }

    void logMessage(const std::string &message) {
        std::ostringstream oss;
        oss << "[" << ros::Time::now() << "] " << message;

        // Log to ROS
        // ROS_INFO_STREAM(oss.str());

        // Log to file
        if (log_file.is_open()) {
            log_file << oss.str() << std::endl;
            log_file.flush();  // Ensure data is written immediately
        } else {
            ROS_WARN("Log file is not open, message not logged to file: %s", message.c_str());
        }
    }

   private:
    std::ofstream log_file;

    void initLogFile(const std::string &folder_name, const std::string &file_name_prefix) {
        std::stringstream log_file_name;
        log_file_name << folder_name << "/" << file_name_prefix << ".txt";
        log_file.open(log_file_name.str());
        if (!log_file.is_open()) {
            ROS_ERROR("Failed to open log file: %s", log_file_name.str().c_str());
        } else {
            ROS_INFO("Logging to file: %s", log_file_name.str().c_str());
        }
    }

    void closeLogFile() {
        if (log_file.is_open()) {
            log_file.close();
        }
    }
};

std::string getCurrentDateTime() {
    auto now = std::chrono::system_clock::now();

    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm now_tm = *std::localtime(&now_c);

    char buffer[80];
    strftime(buffer, sizeof(buffer), "%H-%M", &now_tm);
    std::stringstream ss;
    ss << buffer;
    return ss.str();
}

void ensureDirectoryExists(const std::string &path) {
    struct stat info;
    if (stat(path.c_str(), &info) != 0) {
        if (mkdir(path.c_str(), 0777) != 0) {
            ROS_ERROR("Failed to create directory: %s", path.c_str());
        }
    } else if (!(info.st_mode & S_IFDIR)) {
        ROS_ERROR("Path exists but is not a directory: %s", path.c_str());
    }
}

std::string createLogFolder() {
    std::string folder_name = "/home/bota/catkin_ws_rm/src/quad_agent/logs/";

    std::time_t rawtime = std::time(nullptr);
    struct std::tm *timeinfo = std::localtime(&rawtime);
    char buffer[80];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d", timeinfo);
    folder_name += buffer;
    ensureDirectoryExists(folder_name);

    std::string folder_path = folder_name + "/" + getCurrentDateTime();
    ensureDirectoryExists(folder_path);

    return folder_path;
}
