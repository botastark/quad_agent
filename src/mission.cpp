#include "mission.h"
//Need to WGS84->amsl of gps altitude 
//"When controlling the FCU using global setpoints, you specify the altitude as meters above mean sea level (AMSL). But when sensing the global position, the altitude reported by ~global_position/global is specified as meters above the WGS-84 ellipsoid. This can lead to differences in altitude that are dozens of meters apart."
//https://wiki.ros.org/mavros#mavros.2FPlugins.Avoiding_Pitfalls_Related_to_Ellipsoid_Height_and_Height_Above_Mean_Sea_Level


GeographicLib::Geoid _egm96("egm96-5");  // WARNING: not thread safe
bool reached_target = false;
bool current_gps_received = false;

double calc_geoid_height(double lat, double lon) {
    return _egm96(lat, lon);
}
double amsl_to_ellipsoid_height(double lat, double lon, double amsl) {
  return amsl + GeographicLib::Geoid::GEOIDTOELLIPSOID * calc_geoid_height(lat, lon);
}
double ellipsoid_height_to_amsl(double lat, double lon, double ellipsoid_height) {
  return ellipsoid_height + GeographicLib::Geoid::ELLIPSOIDTOGEOID * calc_geoid_height(lat, lon);
}


void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    current_gps_received = true;
    current_gps.pose.position.altitude = ellipsoid_height_to_amsl(msg->latitude, msg->longitude, msg->altitude);
    current_gps.header.stamp = msg->header.stamp;
    current_gps.pose.position.latitude = msg->latitude;
    current_gps.pose.position.longitude = msg->longitude;
    ROS_INFO_ONCE("Got global position: [%.2f, %.2f, %.2f]", current_gps.pose.position.latitude, current_gps.pose.position.longitude, current_gps.pose.position.altitude);
}

void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void reachedTargetCallback(const std_msgs::Bool::ConstPtr& msg) {    
    reached_target = msg->data;
}

void armDrone(ros::ServiceClient& arming_client) {
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Drone armed");
    } else {
        ROS_ERROR("Failed to arm the drone");
    }
}

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
geographic_msgs::GeoPoseStamped navigateToWaypoint( double latitude, double longitude, double altitude) {
    geographic_msgs::GeoPoseStamped waypoint; // Change message type to geographic_msgs::GeoPoseStamped
    waypoint.header.stamp = ros::Time::now();
    waypoint.header.frame_id = "map"; // Frame should be "map" for GPS waypoints
    waypoint.pose.position.latitude = latitude;
    waypoint.pose.position.longitude = longitude;
    waypoint.pose.position.altitude = altitude; 
    return waypoint;
}

// Function to take a picture
void takePicture(ros::Publisher& take_picture_pub) {
    std_msgs::Bool msg;
    msg.data = true;
    take_picture_pub.publish(msg);
}

//Calculte waypoints local->global
const double EARTH_RADIUS = 6378137.0; // in meters (WGS-84 Earth radius)
const double DEG_TO_RAD = M_PI / 180.0;
const double RAD_TO_DEG = 180.0 / M_PI;

struct GPSPosition {
    double latitude;  // in degrees
    double longitude; // in degrees
    double altitude;  // in meters
};

GPSPosition calculateNewGPSPosition(const geographic_msgs::GeoPoseStamped& currentPos, double dx, double dy, double dz) {
    double lat_rad = currentPos.pose.position.latitude * DEG_TO_RAD;
    double new_latitude = currentPos.pose.position.latitude + (dy / EARTH_RADIUS) * RAD_TO_DEG;
    double new_longitude = currentPos.pose.position.longitude + (dx / (EARTH_RADIUS * cos(lat_rad))) * RAD_TO_DEG;
    double new_altitude = currentPos.pose.position.altitude + dz;

    return {new_latitude, new_longitude, new_altitude};
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "mission_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, stateCallback);
    ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 10, gpsCallback);
    ros::Subscriber mission_complete_sub = nh.subscribe<std_msgs::Bool>("reached_target", 10,reachedTargetCallback);
    
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
    home_position = navigateToWaypoint(current_gps.pose.position.latitude, current_gps.pose.position.longitude, current_gps.pose.position.altitude );
    // ROS_INFO("HOME POSITION");
    // ROS_INFO_STREAM(home_position);

    // std::vector<GPSPosition> waypoints = {
    //     calculateNewGPSPosition(home_position, 0,   0,  2),
    //     calculateNewGPSPosition(home_position, 4,   3,  2),
    //     calculateNewGPSPosition(home_position, -2,  2,  2.5),
    //     calculateNewGPSPosition(home_position, 4,  4,  2),
    //     calculateNewGPSPosition(home_position, 10,  10,  2),
    //     calculateNewGPSPosition(home_position, 0,   0,  1)
    // };
    std::vector<GPSPosition> waypoints = {
        {41.73724768996549, 12.513644919120955, 96},
        {41.73722578686695, 12.513646971647058, 96},
        {41.73720388376838, 12.513649024171759, 95},
        {41.73718198066976, 12.51365107669506, 94},
        {41.7371600775711, 12.513653129216962, 96},
        {41.73713817447241, 12.513655181737462, 95},
        {41.737116271373694, 12.513657234256565, 96},
        {41.73709517590471, 12.513659211092103, 94},
        {41.73707566207436, 12.513700304019201, 94},
        {41.737097565173855, 12.513698251516086, 95}
    };
    ROS_INFO_STREAM("home "<< home_position.pose.position.latitude << ", " << home_position.pose.position.longitude << ", " <<home_position.pose.position.altitude);
    for (const auto& waypoint : waypoints) {
        ROS_INFO_STREAM("waypoint: " << waypoint.latitude << ", " << waypoint.longitude << ", " <<waypoint.altitude);
    }
    double temp_home_alt = home_position.pose.position.altitude ;

    // send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i) {
        home_position.header.stamp = ros::Time::now();
        home_position.pose.position.altitude = temp_home_alt + 2.5;
        global_pos_pub.publish(home_position);
        ros::spinOnce();
        rate.sleep();
    }
    // home_position.pose.position.altitude = temp_home_alt;


    setMode(set_mode_client, "OFFBOARD");
    ROS_INFO("Drone taking off");

    // Wait for takeoff
    while (ros::ok() && current_state.mode != "OFFBOARD") {
        ros::spinOnce();
        rate.sleep();
    }
    armDrone(arming_client);


    // Navigate to each waypoint
    for (const auto& waypoint : waypoints) {
        ROS_INFO_STREAM("Navigating to waypoint: " << std::to_string(waypoint.latitude) << ", " << std::to_string(waypoint.longitude) << ", " <<std::to_string(waypoint.altitude));
        goal_position = navigateToWaypoint(waypoint.latitude, waypoint.longitude, waypoint.altitude);
        global_pos_pub.publish(goal_position);
        // Wait for waypoint reached
        while (ros::ok() && !reached_target) {
            global_pos_pub.publish(goal_position);
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO("reached waypoint");

        // Take picture at waypoint
        takePicture(take_picture_pub);
        ROS_INFO("Taking picture");
        //ros::Duration(5.0).sleep(); // Assuming picture takes 1 second
    }

    // Return to home
    while (ros::ok() && !reached_target) {
        home_position.header.stamp = ros::Time::now();
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
