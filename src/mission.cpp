#include "mission.h"

// Global variables
mavros_msgs::State current_state;
sensor_msgs::NavSatFix current_gps;
bool is_mission_complete = false;

// Callback functions
void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    current_gps = *msg;
}

void missionCompleteCallback(const std_msgs::Bool::ConstPtr& msg) {
    is_mission_complete = msg->data;
}

// Function to arm the drone
void armDrone(ros::ServiceClient& arming_client) {
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Drone armed");
    } else {
        ROS_ERROR("Failed to arm the drone");
    }
}

// Function to set the mode of the drone
void setMode(ros::ServiceClient& set_mode_client, const std::string& mode) {
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = mode;
    if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
        ROS_INFO_STREAM("Mode set to: " << mode);
    } else {
        ROS_ERROR_STREAM("Failed to set mode: " << mode);
    }
}

// Function to navigate to a GPS waypoint
void navigateToWaypoint(ros::Publisher& global_pos_pub, double latitude, double longitude, double altitude) {
    geographic_msgs::GeoPoseStamped waypoint; // Change message type to geographic_msgs::GeoPoseStamped
    waypoint.header.stamp = ros::Time::now();
    waypoint.header.frame_id = "map"; // Frame should be "map" for GPS waypoints
    waypoint.pose.position.latitude = latitude; // Update latitude
    waypoint.pose.position.longitude = longitude; // Update longitude
    waypoint.pose.position.altitude = altitude; // Update altitude
    global_pos_pub.publish(waypoint);
}

// Function to take a picture
void takePicture(ros::Publisher& take_picture_pub) {
    std_msgs::Bool msg;
    msg.data = true;
    take_picture_pub.publish(msg);
}


const double EARTH_RADIUS = 6378137.0; // in meters (WGS-84 Earth radius)
const double DEG_TO_RAD = M_PI / 180.0;
const double RAD_TO_DEG = 180.0 / M_PI;

struct GPSPosition {
    double latitude;  // in degrees
    double longitude; // in degrees
    double altitude;  // in meters
};

GPSPosition calculateNewGPSPosition(const GPSPosition& currentPos, double dx, double dy, double dz) {
    double lat_rad = currentPos.latitude * DEG_TO_RAD;
    double new_latitude = currentPos.latitude + (dy / EARTH_RADIUS) * RAD_TO_DEG;
    double new_longitude = currentPos.longitude + (dx / (EARTH_RADIUS * cos(lat_rad))) * RAD_TO_DEG;
    double new_altitude = currentPos.altitude + dz;

    return {new_latitude, new_longitude, new_altitude};
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "mission_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, stateCallback);
    ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 10, gpsCallback);
    ros::Subscriber mission_complete_sub = nh.subscribe<std_msgs::Bool>("mission_complete", 10, missionCompleteCallback);
    
    ros::Publisher global_pos_pub = nh.advertise<geographic_msgs::GeoPoseStamped>("mavros/setpoint_position/global", 10);
    // ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/global", 10);
    ros::Publisher take_picture_pub = nh.advertise<std_msgs::Bool>("take_picture", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate rate(20.0);
    // Define GPS waypoints
    // std::vector<std::pair<double, double>> waypoints = {
    //     {47.3667, 8.55},      // Fake GPS position
    //     {47.36665, 8.54995},  // Slightly ahead and to the left
    //     {47.36675, 8.55005},  // Slightly behind and to the right
    //     {47.36668, 8.54988},  // Further ahead and to the left
    //     {47.36672, 8.55012}   // Further behind and to the right
    // };
    GPSPosition home_position;
    home_position.latitude = 47.3667;
    home_position.longitude = 8.55;
    home_position.altitude = 537.8749159834465;
    ROS_INFO_STREAM(current_gps.altitude);

    std::vector<GPSPosition> waypoints = {
        calculateNewGPSPosition(home_position, 0,0,5),
        calculateNewGPSPosition(home_position, 1,2,5),
        calculateNewGPSPosition(home_position, -2,2,5),
        calculateNewGPSPosition(home_position, -2,4,3),
        calculateNewGPSPosition(home_position, -2,0,2),
        calculateNewGPSPosition(home_position, 0,0,4)
    };

    // Wait for FCU connection
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

        // send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        // local_pos_pub.publish(poseA);
        navigateToWaypoint(global_pos_pub, current_gps.latitude, current_gps.longitude, current_gps.altitude);         ros::spinOnce();
        rate.sleep();
    }

    // Takeoff
    
    setMode(set_mode_client, "AUTO.TAKEOFF");
    ROS_INFO("Drone taking off");

    // Wait for takeoff
    while (ros::ok() && current_state.mode != "AUTO.TAKEOFF") {
        ros::spinOnce();
        rate.sleep();
    }

    armDrone(arming_client);
    // Navigate to each waypoint

    for (const auto& waypoint : waypoints) {
        navigateToWaypoint(global_pos_pub, waypoint.latitude, waypoint.longitude, waypoint.altitude); // Assuming altitude is 10 meters
    //     navigateToWaypoint(local_pos_pub, waypoint.first, waypoint.second, 10.0); // Assuming altitude is 10 meters
        ROS_INFO_STREAM("Navigating to waypoint: " << waypoint.latitude << ", " << waypoint.longitude << ", " <<waypoint.altitude);

        // Wait for waypoint reached
        while (ros::ok() && !is_mission_complete) {
            
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO("reached waypoint");
        // Take picture at waypoint
        takePicture(take_picture_pub);
        ROS_INFO("Taking picture");
        ros::Duration(1.0).sleep(); // Assuming picture takes 1 second
    }

    // Return to home
    // navigateToWaypoint(local_pos_pub, current_gps.latitude, current_gps.longitude, 0.0); // Land at current GPS position
    navigateToWaypoint(global_pos_pub, current_gps.latitude, current_gps.longitude, current_gps.altitude-10.0); // Land at current GPS position

    ROS_INFO("Returning to home");

    // Wait for landing
    while (ros::ok() && current_state.mode != "AUTO.LAND") {
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Mission complete");
    
    return 0;
}
