#include <unordered_map>

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
    const double R = 6371000.0;
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
std::vector<GPSPosition> readWaypointsFromFile(const std::string &filename, double currentGpsAltitude, std::string altitude_mode) {
    std::vector<GPSPosition> waypoints;
    std::ifstream infile(filename);
    float offset = 0.0;
    if (!infile.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return waypoints;
    }
    if (altitude_mode == "int") {
        offset = currentGpsAltitude;  // asml
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

        alt += offset;
        waypoints.push_back({lat, lon, alt});
    }

    infile.close();
    return waypoints;
}

// http://docs.ros.org/en/noetic/api/mavros_msgs/html/msg/GlobalPositionTarget.html
mavros_msgs::GlobalPositionTarget create_pose(double latitude,
                                              double longitude,
                                              double altitude,
                                              std::string altitude_mode) {
    mavros_msgs::GlobalPositionTarget setpoint;
    setpoint.latitude = latitude;
    setpoint.longitude = longitude;
    setpoint.altitude = altitude;
    if (altitude_mode == "int") {
        setpoint.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_INT;
    } else if (altitude_mode == "rel_alt") {
        setpoint.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_REL_ALT;
    } else if (altitude_mode == "terrain_alt") {
        setpoint.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_TERRAIN_ALT;
    } else {
        setpoint.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_INT;
    }

    setpoint.type_mask = mavros_msgs::GlobalPositionTarget::IGNORE_VX |
                         mavros_msgs::GlobalPositionTarget::IGNORE_VY |
                         mavros_msgs::GlobalPositionTarget::IGNORE_VZ |
                         mavros_msgs::GlobalPositionTarget::IGNORE_AFX |
                         mavros_msgs::GlobalPositionTarget::IGNORE_AFY |
                         mavros_msgs::GlobalPositionTarget::IGNORE_AFZ |
                         mavros_msgs::GlobalPositionTarget::FORCE |
                         mavros_msgs::GlobalPositionTarget::IGNORE_YAW_RATE |
                         mavros_msgs::GlobalPositionTarget::IGNORE_YAW;
    return setpoint;
}

void armDrone(ros::ServiceClient &arming_client) {
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO_ONCE("Drone armed");
    } else {
        ROS_ERROR("Failed to arm the drone");
    }
}

void setMode(ros::ServiceClient &set_mode_client, const std::string &mode) {
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = mode;
    if (set_mode_client.call(offb_set_mode) &&
        offb_set_mode.response.mode_sent) {
        ROS_INFO_STREAM_ONCE("Mode set to: " << mode);
    } else {
        ROS_ERROR_STREAM("Failed to set mode: " << mode);
    }
}

#include <signal.h>

bool g_shutdown_requested = false;

void sigintHandler(int sig) {
    g_shutdown_requested = true;
    ROS_INFO("Shutting down...");
    ros::shutdown();  // Initiate ROS shutdown
}

class Logger {
   public:
    Logger(const std::string &folder_name, const std::string &file_name_prefix) {
        initLogFile(folder_name, file_name_prefix);
    }

    ~Logger() {
        closeLogFile();
    }
    enum LogLevel {
        INFO,
        WARN,
        ERROR
    };

    void logMessage(const std::string &message, LogLevel level = INFO, bool log_once = false) {
        std::ostringstream oss;
        // oss << "[" << ros::Time::now().toSec << "] " << std::fixed << std::setprecision(15) << message;  // Set precision to 6 decimal places
        std::string log_level_prefix;
        if (g_shutdown_requested) {
            return;  // If shutdown requested, do not log further
        }
        switch (level) {
            case INFO:
                log_level_prefix = "INFO";
                break;
            case WARN:
                log_level_prefix = "WARN";
                break;
            case ERROR:
                log_level_prefix = "ERROR";
                break;
            default:
                log_level_prefix = "INFO";  // Default to INFO level if unspecified
                break;
        }

        oss << log_level_prefix << " [" << std::fixed << std::setprecision(9) << ros::Time::now().toSec() << "]: " << std::fixed << std::setprecision(15) << message;

        // Log to file
        if (log_file.is_open()) {
            log_file << oss.str() << std::endl;
            log_file.flush();  // Ensure data is written immediately
        } else {
            ROS_WARN("Log file is not open, message not logged to file: %s", message.c_str());
        }
    }
    void logMessageOnce(const std::string &message, LogLevel level = INFO) {
        if (g_shutdown_requested || logged_messages.count(message) > 0) {
            return;  // If shutdown requested or message already logged, do not log further
        }

        logged_messages.insert(message);
        logMessage(message, level);
    }

   private:
    std::ofstream log_file;
    std::set<std::string> logged_messages;

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

std::string getCurrentDateTime(std::string format) {
    std::time_t rawtime = std::time(nullptr);
    struct std::tm *timeinfo = std::localtime(&rawtime);
    std::stringstream ss;

    char buffer[80];
    if (format == "hm") {
        strftime(buffer, sizeof(buffer), "%H-%M", timeinfo);
        ss << buffer;
    } else if (format == "ymd") {
        strftime(buffer, sizeof(buffer), "%Y-%m-%d", timeinfo);
        ss << buffer;
    } else if (format == "hms-ms") {
        strftime(buffer, sizeof(buffer), "%H-%M-%S", timeinfo);
        auto now = std::chrono::system_clock::now();
        auto ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
        auto fractional_seconds = now - ms;
        ss << buffer << '-' << std::setfill('0') << std::setw(3) << fractional_seconds.count();
    } else {
        strftime(buffer, sizeof(buffer), "%Y-%m-%d", timeinfo);
        ss << buffer;
    }

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

std::string createLogFolder(std::string folder_name) {
    folder_name += getCurrentDateTime("ymd");
    ensureDirectoryExists(folder_name);

    std::string folder_path = folder_name + "/" + getCurrentDateTime("hm");
    ensureDirectoryExists(folder_path);

    return folder_path;
}

std::string to_string_with_precision(double value, int precision = 14) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << value;
    return oss.str();
}

struct ImageMetadata {
    std::string timestamp;
    std::string orientation;
    std::string rpy_orientation;
    std::string local_position;
    std::string global_position;

    void writeToTxt(const std::string &filename) const {
        std::ofstream file(filename, std::ios_base::out | std::ios_base::app);  // Open in append mode
        if (file.is_open()) {
            file << "Timestamp: " << timestamp << std::endl;
            file.flush();
            file << "Global Position (lat, lon, alt): " << global_position << std::endl;
            file << "Local Position (x, y, z): " << local_position << std::endl;
            file.flush();
            file << "Orientation (quaternion): " << orientation << std::endl;
            file.flush();
            file << "Orientation (roll-pitch-yaw): " << rpy_orientation << std::endl;
            file.flush();

            file << "---------------------\n";
            file.flush();
        } else {
            std::cerr << "Failed to open file: " << filename << std::endl;
        }
    }
};

std::unordered_map<std::string, std::string> readConfigFile(const std::string &filename) {
    std::unordered_map<std::string, std::string> config;
    std::ifstream file(filename);
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream line_stream(line);
        std::string key, value;
        if (std::getline(line_stream, key, '=') && std::getline(line_stream, value)) {
            config[key] = value;
        }
    }
    return config;
}
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

using namespace GeographicLib;
using namespace std;

struct SetPoint {
    GPSPosition position_gps;
    // GPSPosition position_end;
    // geometry_msgs::Vector3 x_s;
    // geometry_msgs::Vector3 x_f;
    geometry_msgs::Vector3 position;
    geometry_msgs::Vector3 velocity;      // in meters/s
    geometry_msgs::Vector3 acceleration;  // in meters/s
    double yaw;                           // in rad
    double t;
};

GPSPosition launch_position;
// = {41.73724768996549, 12.513644919120955, 5};

Geocentric earth(Constants::WGS84_a(), Constants::WGS84_f());

geometry_msgs::Vector3 convertGPS2ECEF(GPSPosition positionGPS) {
    geometry_msgs::Vector3 position_ECEF;
    LocalCartesian proj(launch_position.latitude, launch_position.longitude, launch_position.altitude, earth);
    proj.Forward(positionGPS.latitude, positionGPS.longitude, positionGPS.altitude, position_ECEF.x, position_ECEF.y, position_ECEF.z);
    return position_ECEF;
}
GPSPosition convertECEF2GPS(geometry_msgs::Vector3 position_ECEF) {
    GPSPosition positionGPS;

    LocalCartesian proj(launch_position.latitude, launch_position.longitude, launch_position.altitude, earth);
    proj.Reverse(position_ECEF.x, position_ECEF.y, position_ECEF.z, positionGPS.latitude, positionGPS.longitude, positionGPS.altitude);
    // proj.Reverse(x, y, z, lat, lon, h);
    return positionGPS;
}

geometry_msgs::Vector3 convertGPS2NED(GPSPosition positionGPS, geometry_msgs::Vector3 position_origin = convertGPS2ECEF(launch_position)) {
    geometry_msgs::Vector3 position_NED, position_ECEF;
    position_ECEF = convertGPS2ECEF(positionGPS);
    // position_NED.x = position_ECEF.x - position_origin.x;
    // position_NED.y = position_ECEF.y - position_origin.y;
    // position_NED.z = position_ECEF.z - position_origin.z;

    // Calculate the differences in ECEF coordinates
    double dx = position_ECEF.x - position_origin.x;
    double dy = position_ECEF.y - position_origin.y;
    double dz = position_ECEF.z - position_origin.z;

    // Compute the rotation matrix components
    double sin_lat = sin(launch_position.latitude * M_PI / 180.0);
    double cos_lat = cos(launch_position.latitude * M_PI / 180.0);
    double sin_lon = sin(launch_position.longitude * M_PI / 180.0);
    double cos_lon = cos(launch_position.longitude * M_PI / 180.0);

    // Rotation matrix from ECEF to NED
    double R[3][3] = {
        {-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat},
        {-sin_lon, cos_lon, 0},
        {-cos_lat * cos_lon, -cos_lat * sin_lon, -sin_lat}};

    // Apply the rotation matrix to get NED coordinates
    position_NED.x = R[0][0] * dx + R[0][1] * dy + R[0][2] * dz;
    position_NED.y = R[1][0] * dx + R[1][1] * dy + R[1][2] * dz;
    position_NED.z = R[2][0] * dx + R[2][1] * dy + R[2][2] * dz;

    return position_NED;
}

// Function to convert NED position back to GPS
GPSPosition convertNED2GPS(const geometry_msgs::Vector3 &position_NED, geometry_msgs::Vector3 position_origin = convertGPS2ECEF(launch_position)) {
    GPSPosition positionGPS;

    // Compute the rotation matrix components
    double sin_lat = sin(launch_position.latitude * M_PI / 180.0);
    double cos_lat = cos(launch_position.latitude * M_PI / 180.0);
    double sin_lon = sin(launch_position.longitude * M_PI / 180.0);
    double cos_lon = cos(launch_position.longitude * M_PI / 180.0);

    // Rotation matrix from NED to ECEF (transpose of ECEF to NED)
    double R[3][3] = {
        {-sin_lat * cos_lon, -sin_lon, -cos_lat * cos_lon},
        {-sin_lat * sin_lon, cos_lon, -cos_lat * sin_lon},
        {cos_lat, 0, -sin_lat}};

    // Apply the rotation matrix to get ECEF coordinates
    double dx = R[0][0] * position_NED.x + R[0][1] * position_NED.y + R[0][2] * position_NED.z;
    double dy = R[1][0] * position_NED.x + R[1][1] * position_NED.y + R[1][2] * position_NED.z;
    double dz = R[2][0] * position_NED.x + R[2][1] * position_NED.y + R[2][2] * position_NED.z;

    // Convert ECEF coordinates to GPS

    // Calculate ECEF coordinates from NED
    geometry_msgs::Vector3 position_ECEF;
    // Calculate the ECEF coordinates
    position_ECEF.x = position_origin.x + dx;
    position_ECEF.y = position_origin.y + dy;
    position_ECEF.z = position_origin.z + dz;
    // position_ECEF.x = position_origin.x + position_NED.x;
    // position_ECEF.y = position_origin.y + position_NED.y;
    // position_ECEF.z = position_origin.z + position_NED.z;  // NED is negative down
    positionGPS = convertECEF2GPS(position_ECEF);
    return positionGPS;
}

void getArcLength(double t, double L, double &sigma, double &dsigma, double &ddsigma) {
    // trapezoid velocity timing
    double Ts = v_max / a_max;
    double T = (L * a_max + v_max * v_max) / a_max / v_max;
    if (t >= 0 && t < Ts) {
        sigma = a_max * t * t / 2;
        dsigma = a_max * t;
        ddsigma = a_max;
    } else if (t > Ts && t < (T - Ts)) {
        sigma = v_max * t - v_max * v_max / 2 / a_max;
        dsigma = v_max;
        ddsigma = 0;
    } else if (t > (T - Ts) && t < T) {
        sigma = -a_max * (t - T) * (t - T) / 2 + v_max * t - v_max * v_max / a_max;
        dsigma = -a_max * t;
        ddsigma = -a_max;
    } else {
        sigma = L;
        dsigma = 0;
        ddsigma = 0;
    }
}
void calculateTrajectory_axis(double r0, double rf, double L, double t, double &position, double &velocity, double &acceleration) {
    double sigma, dsigma, ddsigma;
    getArcLength(t, L, sigma, dsigma, ddsigma);
    position = r0 + 3 * (rf - r0) / L / L * sigma * sigma - 2 * (rf - r0) / L / L / L * sigma * sigma * sigma;
    velocity = 6 / L / L * (rf - r0) * (sigma * dsigma - sigma * sigma * dsigma);
    acceleration = 6 / L / L * (rf - r0) * (dsigma * dsigma + sigma * ddsigma - 2 * sigma * dsigma - sigma * sigma * ddsigma);
}

double calculateDistance(geometry_msgs::Vector3 pos_s, geometry_msgs::Vector3 pos_f) {
    geometry_msgs::Vector3 distances;
    distances.x = std::abs(pos_f.x - pos_s.x);  // Absolute distance in North (x)
    distances.y = std::abs(pos_f.y - pos_s.y);  // Absolute distance in East (y)
    distances.z = std::abs(pos_f.z - pos_s.z);
    double eucledian_dist = distances.x * distances.x + distances.y * distances.y + distances.z * distances.z;
    eucledian_dist = std::sqrt(eucledian_dist);
    return eucledian_dist;
}

SetPoint generateTrajectory(GPSPosition start, GPSPosition end, double time) {
    // Convert GPS coordinates to local Cartesian coordinates
    geometry_msgs::Vector3 pos_s = convertGPS2NED(start);
    geometry_msgs::Vector3 pos_f = convertGPS2NED(end);

    SetPoint out;
    out.t = time;

    double L = calculateDistance(pos_s, pos_f);

    ROS_INFO_STREAM("distance L: " << L);

    calculateTrajectory_axis(pos_s.x, pos_f.x, L, time, out.position.x, out.velocity.x, out.acceleration.x);
    calculateTrajectory_axis(pos_s.y, pos_f.y, L, time, out.position.y, out.velocity.y, out.acceleration.y);
    calculateTrajectory_axis(pos_s.z, pos_f.z, L, time, out.position.z, out.velocity.z, out.acceleration.z);
    out.yaw = std::atan2(out.velocity.y, out.velocity.x);
    out.position_gps = convertNED2GPS(out.position);
    return out;
}

mavros_msgs::GlobalPositionTarget create_pose_traj(SetPoint target, std::string altitude_mode) {
    mavros_msgs::GlobalPositionTarget setpoint;

    if (altitude_mode == "int") {
        setpoint.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_INT;
    } else if (altitude_mode == "rel_alt") {
        setpoint.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_REL_ALT;
    } else if (altitude_mode == "terrain_alt") {
        setpoint.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_TERRAIN_ALT;
    } else {
        setpoint.coordinate_frame = mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_INT;
    }

    setpoint.latitude = target.position_gps.latitude;
    setpoint.longitude = target.position_gps.longitude;
    setpoint.altitude = target.position_gps.altitude;
    setpoint.velocity.x = target.velocity.x;
    setpoint.velocity.y = target.velocity.y;
    setpoint.velocity.z = target.velocity.z;
    setpoint.acceleration_or_force.x = target.acceleration.x;
    setpoint.acceleration_or_force.y = target.acceleration.y;
    setpoint.acceleration_or_force.z = target.acceleration.z;
    // setpoint.yaw = target.yaw;

    setpoint.type_mask =
        // mavros_msgs::GlobalPositionTarget::IGNORE_VX |
        // mavros_msgs::GlobalPositionTarget::IGNORE_VY |
        // mavros_msgs::GlobalPositionTarget::IGNORE_VZ |
        // mavros_msgs::GlobalPositionTarget::IGNORE_AFX |
        // mavros_msgs::GlobalPositionTarget::IGNORE_AFY |
        // mavros_msgs::GlobalPositionTarget::IGNORE_AFZ |
        // mavros_msgs::GlobalPositionTarget::FORCE |
        mavros_msgs::GlobalPositionTarget::IGNORE_YAW_RATE |
        mavros_msgs::GlobalPositionTarget::IGNORE_YAW;
    return setpoint;
}