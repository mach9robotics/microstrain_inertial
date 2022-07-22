#include <ros/ros.h>
#include <string>
#include <nav_msgs/Odometry.h>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include "microstrain_inertial_msgs/FilterStatus.h"
#include <visualization_msgs/Marker.h>

// double lat = 40.46;
// double lon = -79.96;
// double alt = 244.86;

static GeographicLib::LocalCartesian odom_converter;
static bool is_initial = true;
static int filter_status;
ros::Publisher odom_pub;
ros::Publisher start_marker_pub;

static visualization_msgs::Marker marker;



void odom_cb(const nav_msgs::Odometry::ConstPtr& odom_llc)
{
    if (filter_status != 4)
    {
        return;
    }
    double lat = odom_llc->pose.pose.position.y;
    double lon = odom_llc->pose.pose.position.x;
    double alt = odom_llc->pose.pose.position.z;
    double x,y,z;

    if (is_initial == true)
    {
        odom_converter.Reset(lat, lon, alt);
        odom_converter.Forward(lat, lon, alt, x, y, z);
        marker.header.stamp = ros::Time();
        marker.id += 1;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;
        marker.pose.orientation.x = odom_llc->pose.pose.orientation.x;
        marker.pose.orientation.y = odom_llc->pose.pose.orientation.y;
        marker.pose.orientation.z = odom_llc->pose.pose.orientation.z;
        marker.pose.orientation.w = odom_llc->pose.pose.orientation.w;
        start_marker_pub.publish(marker);
        ROS_WARN_STREAM("Lat: " << lat << "Lon: " << lon << "Alt: " << alt);
        ROS_WARN_STREAM("X: " << x << "Y: " << y << "Z: " << z);
        is_initial = false;
    }
    odom_converter.Forward(lat, lon, alt, x, y, z);
    nav_msgs::Odometry odom_cart = *odom_llc;
    odom_cart.pose.pose.position.x = x;
    odom_cart.pose.pose.position.y = y;
    odom_cart.pose.pose.position.z = z;
    odom_cart.child_frame_id = "sensor_cartesian";
    odom_pub.publish(odom_cart);
    return;
}

void filter_status_cb(const microstrain_inertial_msgs::FilterStatus::ConstPtr& status_msg)
{
    filter_status = status_msg->filter_state;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv, "odom_converter");
    std::string deviceName;
    ros::NodeHandle nh;

    //initialize a start marker
    marker.header.frame_id = "sensor_wgs84";
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 1.0;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    ros::Subscriber odom_sub = nh.subscribe("/GQ7/nav/odom",10,odom_cb);
    ros::Subscriber filter_status_sub = nh.subscribe("/GQ7/nav/status",10,filter_status_cb);

    odom_pub = nh.advertise<nav_msgs::Odometry>("/GQ7/nav/odom_cartesian", 10);
    start_marker_pub = nh.advertise<visualization_msgs::Marker>("/GQ7/nav/start_marker",10);
    
    ros::spin();
    return 0;
}
