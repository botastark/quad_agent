#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <geographic_msgs/GeoPoseStamped.h>
#include <cmath>
#include <GeographicLib/Geoid.hpp>

geographic_msgs::GeoPoseStamped current_gps;
mavros_msgs::State current_state;
