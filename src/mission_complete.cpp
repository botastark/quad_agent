#include "mission.h"
#include <std_msgs/Bool.h>
#include <GeographicLib/Geoid.hpp>


GeographicLib::Geoid _egm96("egm96-5");  // WARNING: not thread safe
geographic_msgs::GeoPoseStamped current_gps;
// Global variables
mavros_msgs::State current_state;

// sensor_msgs::NavSatFix current_gps;
std_msgs::Bool is_mission_complete;

geometry_msgs::Vector3 current_target_global;
double tolerance = 0.05;



bool current_gps_received = false;
// sensor_msgs::NavSatFix current_gps;

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
    ROS_INFO_ONCE("Got global position: [%.2f, %.2f, %.2f]", current_gps.pose.position.latitude, current_gps.pose.position.longitude, current_gps.pose.position.altitude);
}



// Callback functions
void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

// void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
//     current_gps = *msg;
// }

void targetCallback(const geographic_msgs::GeoPoseStamped::ConstPtr& msg) {
    geographic_msgs::GeoPoseStamped waypoint = *msg;
    current_target_global.x = waypoint.pose.position.latitude; // Update latitude
    current_target_global.y = waypoint.pose.position.longitude; // Update longitude
    current_target_global.z = waypoint.pose.position.altitude; // Update altitude
}


std_msgs::Bool missionComplete() {
    double dx = current_target_global.x - current_gps.pose.position.latitude;
    double dy = current_target_global.y - current_gps.pose.position.longitude;
    double dz = current_target_global.z - current_gps.pose.position.altitude;
    double dist = sqrt(dx * dx + dy * dy + dz * dz);
    if (dist > tolerance || dist==0.0 || dist ==0){
	    is_mission_complete.data=false;
        ROS_INFO_STREAM("dist: "<<dist<<" dx: "<<dx <<" dy: "<<dy <<" dz: "<<dz);
    }else{
	    is_mission_complete.data=true;
        ROS_INFO("Reached waypoint!");
    }
    
    return is_mission_complete;

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "missionchecker_node");
    ros::NodeHandle nh_;

    ros::Subscriber state_sub = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, stateCallback);
    ros::Subscriber gps_sub = nh_.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 10, gpsCallback);
    ros::Publisher mission_complete_pub = nh_.advertise<std_msgs::Bool>("mission_complete", 10);
    ros::Subscriber target_pos_sub = nh_.subscribe<geographic_msgs::GeoPoseStamped>("mavros/setpoint_position/global", 10, targetCallback);
    ros::Rate rate(20.0);

    while(ros::ok()){
        mission_complete_pub.publish(missionComplete());
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
    }
