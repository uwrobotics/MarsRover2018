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



    return 0;
}