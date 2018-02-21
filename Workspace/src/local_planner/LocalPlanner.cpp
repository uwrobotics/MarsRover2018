#include "DynamicWindow.h"
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Twist.h"

ros::Publisher velocityPub;

void CurrentVelocityCallback(geometry_msgs::Twist::ConstPtr& currentVelocity)
{

}

void OccupancyGridCallback(nav_msgs::OccupancyGrid::ConstPtr& occupancyGrid)
{

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "local_planner");
    ros::NodeHandle nh;
    ros::Subscriber mapSub = nh.subscribe("occupancy_grid",1,OccupancyGridCallback);
    ros::Subscriber velSub = nh.subscribe("current_vel",1,CurrentVelocityCallback);
    velocityPub = nh.advertise<geometry_msgs::Twist>("local_planner_vel",1);


    return 0;
}