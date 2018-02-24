#include "DynamicWindow.h"
#include "LocalPlanner.h"
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

/*    RobotParams_t roverParams = {
            maxAngAccel : 1.0,
            maxLinAccel : 1.0,
            maxLinDecel : 1.0,
            maxV : 1.5,
            minV : 0.0,
            maxW : 1.0,
            robotLength : 1.5,
            robotWidth : 0.5
    };*/
    //CLocalPlanner localPlanner(&nh, roverParams);

    ros::spin();


    return 0;
}