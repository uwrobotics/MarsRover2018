#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <cmath>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <robot_localization/navsat_conversions.h>

#define MAX_ANTENNA_FIXES   10

double antenna_x = 0;                           // Positive to the right (eastward)
double antenna_x_sum = 0; 
double antenna_y = 0;                           // Positive facing up (northward)
double antenna_y_sum = 0; 
double rover_x = 0;                             // Positive to the right (eastward)
double rover_y = 0;                             // Positive facing up (northward)
uint8_t num_of_antenna_pings = 0;

void antenna_loc_callback(const sensor_msgs::NavSatFix::ConstPtr& ptr)
{
    // Average the first few fixes to get a more accurate permanent location of the antenna
    if (num_of_antenna_pings < MAX_ANTENNA_FIXES) 
    {
        std::string UTMZone;
        double utm_east, utm_north;
        RobotLocalization::NavsatConversions::LLtoUTM(ptr->latitude, ptr->longitude,
                utm_north, utm_east, UTMZone); // Converts lat/long to UTM east/west

        antenna_x_sum += utm_east;
        antenna_y_sum += utm_north;

        antenna_x = antenna_x_sum / ++num_of_antenna_pings;
        antenna_y = antenna_y_sum / num_of_antenna_pings;
    }
}

void rover_loc_callback(const sensor_msgs::NavSatFix::ConstPtr& ptr)
{
    if (num_of_antenna_pings == 0)
    {
        // Haven't received antenna location yet, do nothing
        return;
    }
    else 
    {
        // Find dx and dy based on antenna locations and rover locations, and use arctan to find angle.
        // Assume antenna is pointing north?
        std::string UTMZone;
        double utm_east, utm_north;
        RobotLocalization::NavsatConversions::LLtoUTM(ptr->latitude, ptr->longitude,
                utm_north, utm_east, UTMZone); // Converts lat/long to UTM east/west

        double dx = antenna_x - rover_x;
        double dy = antenna_y - rover_y;

        double angle = atan (dy/dx); // Double check this
        // Either publish this to pi, or have the pi provide a function that this node can call
    }
}

int32_t main (int argc, char *argv[])
{
    ros::init(argc, argv, "antenna_mover");
    ros::NodeHandle node_handle;

    // TODO put these in rosparams
    ros::Subscriber antenna_sub = node_handle.subscribe("antenna/fix", 1, antenna_loc_callback);
    ros::Subscriber rover_sub = node_handle.subscribe("/navsat/fix", 1, rover_loc_callback);

    ros::Rate loop_rate(10); // 20Hz update rate
    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce(); // Check for new messages
    }
}