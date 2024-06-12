#include "mission.h"


GeographicLib::Geoid _egm96("egm96-5");  // WARNING: not thread safe
std_msgs::Bool reached_target;
geometry_msgs::Vector3 current_target_global;


double tolerance = 0.2;
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

// Callback functions

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    //current_gps = *msg;
    current_gps_received = true;
    current_gps.pose.position.altitude = ellipsoid_height_to_amsl(msg->latitude, msg->longitude, msg->altitude);
    current_gps.header.stamp = msg->header.stamp;
    current_gps.pose.position.latitude = msg->latitude;
    current_gps.pose.position.longitude = msg->longitude;
}


// Callback functions
void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}


void targetCallback(const geographic_msgs::GeoPoseStamped::ConstPtr& msg) {
    geographic_msgs::GeoPoseStamped waypoint = *msg;
    current_target_global.x = waypoint.pose.position.latitude; // Update latitude
    current_target_global.y = waypoint.pose.position.longitude; // Update longitude
    current_target_global.z = waypoint.pose.position.altitude; // Update altitude
}


double haversine(double lat1, double lon1, double lat2, double lon2) {
    // Constants
    const double R = 6371000.0; // Earth radius in meters

    // Convert latitude and longitude from degrees to radians
    double lat1_rad = lat1 * M_PI / 180.0;
    // double lon1_rad = lon1 * M_PI / 180.0;
    double lat2_rad = lat2 * M_PI / 180.0;
    // double lon2_rad = lon2 * M_PI / 180.0;

    // Haversine formula
    double dlat = (lat2-lat1 )* M_PI / 360.0;
    double dlon = (lon2-lon1 )* M_PI / 360.0;

    double a = std::sin(dlat) * std::sin(dlat) +
               std::cos(lat1_rad) * std::cos(lat2_rad) *
               std::sin(dlon) * std::sin(dlon);

    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

    double distance = R * c;

    return distance;
}


double calculateDistance(const geographic_msgs::GeoPoseStamped& a, const geographic_msgs::GeoPoseStamped& b) {
    double lat1 = a.pose.position.latitude;
    double lon1 = a.pose.position.longitude;
    double lat2 = b.pose.position.latitude;
    double lon2 = b.pose.position.longitude;

    return haversine(lat1, lon1, lat2, lon2);
}



std_msgs::Bool missionComplete() {
    // double dx = current_target_global.x - current_gps.pose.position.latitude;
    // double dy = current_target_global.y - current_gps.pose.position.longitude;
    // double dz = current_target_global.z - current_gps.pose.position.altitude;
    // double dist = sqrt(dx * dx + dy * dy + dz * dz);
    ROS_INFO_STREAM("curr gps: "<<current_gps.pose.position.altitude <<" target h: "<<current_target_global.z );
    double vert_dist = haversine(current_target_global.x, current_target_global.y , current_gps.pose.position.latitude, current_gps.pose.position.longitude);
    double hori_dist = current_gps.pose.position.altitude - current_target_global.z ;
    double dist = sqrt(vert_dist * vert_dist + hori_dist * hori_dist);
    if (dist > tolerance || dist==0.0 || dist ==0){
	    reached_target.data=false;
        // ROS_INFO_STREAM("dist: "<<dist<<" dx: "<<dx <<" dy: "<<dy <<" dz: "<<dz);
        ROS_INFO_STREAM("dist: "<<dist<<" vert_dist: "<<vert_dist <<" hori_dist: "<<hori_dist);
    }else{
	    reached_target.data=true;
        ROS_INFO("Reached waypoint!");
    }
    
    return reached_target;

}




int main(int argc, char **argv) {
    ros::init(argc, argv, "missionchecker_node");
    ros::NodeHandle nh_;

    ros::Subscriber state_sub = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, stateCallback);
    ros::Subscriber gps_sub = nh_.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 10, gpsCallback);
    ros::Publisher reached_target_pub = nh_.advertise<std_msgs::Bool>("reached_target", 10);
    ros::Subscriber target_pos_sub = nh_.subscribe<geographic_msgs::GeoPoseStamped>("mavros/setpoint_position/global", 10, targetCallback);
    ros::Rate rate(20.0);
    // while (ros::ok() && current_state.mode != "AUTO.TAKEOFF") {
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    while(ros::ok()){
        reached_target_pub.publish(missionComplete());
        ros::spinOnce();
        rate.sleep();
    }
    // geographic_msgs::GeoPoseStamped pointA;
    // geographic_msgs::GeoPoseStamped pointB;
// [ INFO] [1716278666.215440676, 82.771000000]: home 47.3977, 8.54552, 488.375
// [ INFO] [1716278666.215453741, 82.771000000]: waypoint: 47.3977, 8.54552, 490.875
// [ INFO] [1716278666.215464602, 82.771000000]: waypoint: 47.3977, 8.54553, 490.875
// [ INFO] [1716278666.215475773, 82.771000000]: waypoint: 47.3977, 8.54549, 490.875
// [ INFO] [1716278666.215491693, 82.771000000]: waypoint: 47.3977, 8.54549, 491.375
// [ INFO] [1716278666.215506542, 82.771000000]: waypoint: 47.3977, 8.54549, 490.375
// [ INFO] [1716278666.215530477, 82.771000000]: waypoint: 47.3977, 8.54552, 492.375
    // // Example coordinates with high precision
    // pointA.pose.position.latitude = 40.748817; // Example: Point near PointA
    // pointA.pose.position.longitude = -73.985428;
    // pointB.pose.position.latitude = 40.748817; // Example: Point exactly the same as PointA
    // pointB.pose.position.longitude = -73.985428;

    // // Print the coordinates being used
    // std::cout << std::fixed << std::setprecision(15);
    // std::cout << "Point A: (" << pointA.pose.position.latitude << ", " << pointA.pose.position.longitude << ")\n";
    // std::cout << "Point B: (" << pointB.pose.position.latitude << ", " << pointB.pose.position.longitude << ")\n";

    // double distance = calculateDistance(pointA, pointB);
    // ROS_INFO("Distance between points: %.15f meters", distance);


    return 0;
    }
