#include "mission.h"
// Global variables
mavros_msgs::State current_state;
sensor_msgs::NavSatFix current_gps;
bool is_mission_complete = false;
geometry_msgs::Vector3 current_target_global;
double tolerance = 0.001;

// Callback functions
void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    current_gps = *msg;
}

void targetCallback(const geographic_msgs::GeoPoseStamped::ConstPtr& msg) {
    geographic_msgs::GeoPoseStamped waypoint = *msg;
    current_target_global.x = waypoint.pose.position.latitude; // Update latitude
    current_target_global.y = waypoint.pose.position.longitude; // Update longitude
    current_target_global.z = waypoint.pose.position.altitude; // Update altitude
}


bool missionComplete() {
    double dx = current_target_global.x - current_gps.latitude;
    double dy = current_target_global.y - current_gps.longitude;
    double dz = current_target_global.z - current_gps.altitude;
    double dist = sqrt(dx * dx + dy * dy + dz * dz);
    if (dist > tolerance){
        return false;
        //mission_complete_pub.publish(false);
    }else{
        return true;
        //mission_complete_pub.publish(true);
    }

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "missionchecker_node");
    ros::NodeHandle nh_;

    ros::Subscriber state_sub = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, stateCallback);
    ros::Subscriber gps_sub = nh_.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 10, gpsCallback);
    ros::Publisher mission_complete_pub = nh_.advertise<std_msgs::Bool>("mission_complete", 10);
    ros::Subscriber target_pos_sub = nh_.subscribe<geographic_msgs::GeoPoseStamped>("mavros/setpoint_position/global", 10, targetCallback);
    ros::Rate rate(20.0);

    while(ros:ok()){
        mission_complete_pub.publish(missionComplete());
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
    }