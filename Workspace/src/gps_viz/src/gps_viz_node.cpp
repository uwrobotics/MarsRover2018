//
// Created by tom on 01/06/18.
//

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <robot_localization/navsat_conversions.h>
#include <visualization_msgs/Marker.h>


double originX = 0;
double originY = 0;
bool bFirst = true;

std::string utmzone;


ros::Publisher* pGoalPub = nullptr;
ros::Publisher* pCurPub = nullptr;

void publishMarker(double x, double y, bool bCur)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world_frame";
  marker.header.stamp = ros::Time();
  marker.ns = "gps_viz";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1;
  marker.scale.y = 1;
  marker.scale.z = 1;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  if (bCur) {
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    pCurPub->publish(marker);
  }
  else {
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    pGoalPub->publish(marker);
  }
}


void CurGpsCallback(sensor_msgs::NavSatFixConstPtr gps)
{
  double x, y;
  //m_goalGPS = *goal;
  RobotLocalization::NavsatConversions::LLtoUTM(gps->latitude,
                                                gps->longitude,
                                                y,
                                                x,
                                                utmzone);

  if (bFirst)
  {
    originX = x;
    originY = y;
    bFirst = false;

  }

  publishMarker(x - originX,y - originY,true);
}


void GoalGpsCallback(sensor_msgs::NavSatFixConstPtr gps)
{
  double x, y;
  //m_goalGPS = *goal;
  RobotLocalization::NavsatConversions::LLtoUTM(gps->latitude,
                                                gps->longitude,
                                                y,
                                                x,
                                                utmzone);

  if (bFirst)
  {
    originX = x;
    originY = y;
    bFirst = false;

  }

  publishMarker(x - originX,y - originY,false);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "gps_viz");
  ros::NodeHandle nh;

  ros::Subscriber cursub = nh.subscribe("/fix",1,CurGpsCallback);
  ros::Subscriber goalsub = nh.subscribe("/goal_gps",1,GoalGpsCallback);
  ros::Publisher curPub = nh.advertise<visualization_msgs::Marker>("/gps_viz/cur_gps",1);
  ros::Publisher goalPub = nh.advertise<visualization_msgs::Marker>("/gps_viz/goal_gps",1);
  pCurPub = &curPub;
  pGoalPub = &goalPub;



  ros::Rate rate(10);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

}

