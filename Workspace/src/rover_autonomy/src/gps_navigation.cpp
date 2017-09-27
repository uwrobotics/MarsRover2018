#include <ros/ros.h>
#include <robot_localization/navsat_conversions.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <sensor_msgs/NavSatFix.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/String.h"
#include <string>
#include <cmath>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <rover_autonomy/gps_coord.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int32MultiArray.h>

// NOTE: Make sure the "broadcast_utm_transform" parameter is set to "true" in localization.launch

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
const int MAX_LENGTH=5;
ros::Publisher waypoints_viz_pub, divisions_viz_pub;

bool received_new_point = false;
std::vector<double> way_pts_utm_east;
std::vector<double> way_pts_utm_north;
int num_waypoints = 0;
int ball_bearing = 0, ball_dist = 0;
bool ball_valid = false;

void dividePath(double destX, double destY, double currX, double currY, std::vector<double> &divideX, std::vector<double> &divideY, int divisions){
    for (int x=1; x<=divisions; x++){
        divideX[x-1]=currX*(1-(x*1.0)/divisions) + destX * (x*1.0)/divisions;
        divideY[x-1]=currY*(1-(x*1.0)/divisions) + destY * (x*1.0)/divisions;
    }
    ROS_INFO("Generated waypoints to target");
}


void move(double UTMEast, double UTMNorth){
    MoveBaseClient ac("move_base", true);
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    // Get position of base_link wrt global utm coordinate system

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped baseLinkToUTM;
    try{
            baseLinkToUTM = tfBuffer.lookupTransform("utm", "base_link", ros::Time(0), ros::Duration(2));
    }
    catch (tf2::TransformException ex ){
            ROS_ERROR("%s",ex.what());
    }

    ROS_INFO("Generated transform from base_link to utm");
    double currX = baseLinkToUTM.transform.translation.x;
    double currY = baseLinkToUTM.transform.translation.y;

    int divisions = (int) ((pow ((pow((UTMEast-currX),2)+pow((UTMNorth-currY),2)),0.5))/MAX_LENGTH) + 1;

    std::vector<double> xTargets (divisions);
    std::vector<double> yTargets (divisions);

    dividePath(UTMEast, UTMNorth, currX, currY, xTargets, yTargets, divisions);

    // Move rover sequentially to each waypoint

    ROS_WARN("Moving to next waypoint... (%d divisions)", divisions);

    visualization_msgs::Marker divisions_viz;
    divisions_viz.header.frame_id = "utm";
    divisions_viz.id = 0;
    divisions_viz.type = visualization_msgs::Marker::SPHERE_LIST;
    divisions_viz.action = visualization_msgs::Marker::ADD;
    divisions_viz.ns = "goal_waypoints";
    divisions_viz.scale.x = 0.3;
    divisions_viz.scale.y = 0.3;
    divisions_viz.scale.z = 0.3;
    divisions_viz.color.r = 1.0;
    divisions_viz.color.g = 0.0;
    divisions_viz.color.b = 0.0;
    divisions_viz.color.a = 1.0;

    // publish the divisions list
    for (int i = 0; i < divisions; i++) {
        geometry_msgs::Point pt_viz;
        pt_viz.x = xTargets[i];
        pt_viz.y = yTargets[i];
        pt_viz.z = 0;
        divisions_viz.points.push_back(pt_viz);
    }

    divisions_viz_pub.publish(divisions_viz);

    for (int i=0; i < divisions && ros::ok(); i++) {

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "utm";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = xTargets[i];
        goal.target_pose.pose.position.y = yTargets[i];
        goal.target_pose.pose.orientation.w = 1.0;


        std::stringstream ss;
        std_msgs::String msg;
        double xTemp = xTargets[i];
        double yTemp = yTargets[i];
        ss << "xTarget: " << xTemp << ", yTarget: " << yTemp << ", divisions: " << divisions;
        ROS_INFO("Sending goal (x,y) = (%.2f, %.2f)", xTemp, yTemp);
        msg.data = ss.str();
        ac.sendGoal(goal);

        ac.waitForResult();
    }
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_WARN("Completed move_base nav to waypoint");
    else
        ROS_ERROR("Failed move move_base nav waypoint");
}


void receiveMessage(const rover_autonomy::gps_coord::ConstPtr& ptr){
    // std::vector<double> UTMEasts(ptr->length);
    // std::vector<double> UTMNorths(ptr->length);
    num_waypoints = ptr->length;

    visualization_msgs::Marker waypoints_viz;
    waypoints_viz.header.frame_id = "utm";
    waypoints_viz.id = 1;
    waypoints_viz.type = visualization_msgs::Marker::SPHERE_LIST;
    waypoints_viz.action = visualization_msgs::Marker::ADD;
    waypoints_viz.ns = "goal_waypoints";
    waypoints_viz.scale.x = 1;
    waypoints_viz.scale.y = 1;
    waypoints_viz.scale.z = 1;
    waypoints_viz.color.r = 0.0;
    waypoints_viz.color.g = 1.0;
    waypoints_viz.color.b = 0.0;
    waypoints_viz.color.a = 1.0;

    for (int i = 0; i < ptr->length; i++) {
        std::string UTMZone;
        double utm_east, utm_north;
        RobotLocalization::NavsatConversions::LLtoUTM(ptr->latitudes[i], ptr->longitudes[i],
            utm_north, utm_east, UTMZone); // Converts lat/long to UTM east/west

        way_pts_utm_east.push_back(utm_east);
        way_pts_utm_north.push_back(utm_north);

        geometry_msgs::Point pt_viz;
        pt_viz.x = utm_east;
        pt_viz.y = utm_north;
        pt_viz.z = 0;
        waypoints_viz.points.push_back(pt_viz);
    }

    waypoints_viz_pub.publish(waypoints_viz);

    received_new_point = true;
}

void tennisball_callback(const std_msgs::Int32MultiArray data) {
    if (data.data[0]) {
        ball_valid = true;
    } else {
        ball_valid = false;
    }

    ball_bearing = data.data[4];
    ball_dist = data.data[3];
}

main(int argc, char** argv){
    ros::init(argc, argv, "gps_navigation");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/rover_autonomy/coordinates", 1, receiveMessage);
    ros::Subscriber tennisball_sub = n.subscribe("/tennisball", 1, tennisball_callback);
    ros::Publisher gimbal_pub = n.advertise<std_msgs::Int32MultiArray>("gimbal_cmd", 1, true);
    waypoints_viz_pub = n.advertise<visualization_msgs::Marker>("waypoints_viz", 1, true);
    divisions_viz_pub = n.advertise<visualization_msgs::Marker>("divisions_viz", 1, true);

    ros::Rate loop_rate(10);    //20Hz update rate

    const int STATE_WAITING_FOR_WAYPOINTS = 0, 
              STATE_MOVE_BASE_NAV = 1,
              STATE_TENNIS_BALL_SEARCH = 2,
              STATE_TENNIS_BALL_NAV_YAW = 3,
              STATE_TENNIS_BALL_NAV_DIST = 4,
              STATE_TEAR_DOWN = 5;
    int state = STATE_WAITING_FOR_WAYPOINTS;
    int idx_waypoint = 0;

    double yaw_offset, dist_offset;

    while (ros::ok()) {
        loop_rate.sleep(); //Maintain the loop rate
        ros::spinOnce();   //Check for new messages
        
        if (state == STATE_WAITING_FOR_WAYPOINTS) {
            ROS_WARN("state = STATE_WAITING_FOR_WAYPOINTS");

            if (received_new_point) {
                idx_waypoint = 0;

                received_new_point = false;
                state = STATE_MOVE_BASE_NAV;
                ROS_WARN("Switching to STATE_MOVE_BASE_NAV");
            }
        }

        else if (state == STATE_MOVE_BASE_NAV) {
            ROS_WARN("state = STATE_MOVE_BASE_NAV");
            move(way_pts_utm_east[idx_waypoint], way_pts_utm_north[idx_waypoint]);

            state = STATE_TENNIS_BALL_SEARCH;
            ROS_WARN("Switching to STATE_TENNIS_BALL_SEARCH");

            // state = STATE_TEAR_DOWN;
            // ROS_WARN("Switching to STATE_TEAR_DOWN");
        }

        else if (state == STATE_TENNIS_BALL_SEARCH) {
            ROS_WARN("state = STATE_TENNIS_BALL_SEARCH");

            bool ball_valid_cpy = false;
            double ball_dist_cpy = 0, ball_bearing_cpy = 0;

            int gimbal_angle;
            for (gimbal_angle = -180; gimbal_angle < 180; gimbal_angle+=30) {
                //publish to 
                std_msgs::Int32MultiArray pub_data;
                pub_data.data = std::vector<int>(2);

                pub_data.data[0] = -5;
                pub_data.data[0] = gimbal_angle;

                gimbal_pub.publish(pub_data);

                // wait for 2 seconds
                // ros::Duration(2).sleep();

                // read the ball position
                if (ball_valid) {
                    ball_valid_cpy = ball_valid;
                    ball_dist_cpy = ball_dist;
                    ball_bearing_cpy = ball_bearing;
                    break;
                }
            }

            if (!ball_valid_cpy) {
                // cannot find the ball, assume we are at the target
                idx_waypoint++;
                if (idx_waypoint >= num_waypoints) {
                    state = STATE_TEAR_DOWN;
                    ROS_WARN("Switching to STATE_TEAR_DOWN");
                } else {
                    state = STATE_MOVE_BASE_NAV;
                    ROS_WARN("Switching to STATE_MOVE_BASE_NAV");
                }
            } 
            else {
                yaw_offset = gimbal_angle * M_PI / 180 + ball_bearing_cpy;
                dist_offset = ball_dist_cpy;
                state = STATE_TENNIS_BALL_NAV_YAW;
                ROS_WARN("Switching to STATE_TENNIS_BALL_NAV_YAW");
            }
        }

        else if (state == STATE_TENNIS_BALL_NAV_YAW) {
            ROS_WARN("state = STATE_TENNIS_BALL_NAV_YAW");
            state = STATE_TENNIS_BALL_NAV_YAW;
            ROS_WARN("Switching to STATE_TENNIS_BALL_NAV_YAW");
        }

        else if (state == STATE_TENNIS_BALL_NAV_DIST) {
            ROS_WARN("state = STATE_TENNIS_BALL_NAV_DIST");
            state = STATE_TEAR_DOWN;
            ROS_WARN("Switching to STATE_TEAR_DOWN");
        }

        else if (state == STATE_TEAR_DOWN) {
            ROS_WARN("state = STATE_TEAR_DOWN");
            idx_waypoint = 0;
            way_pts_utm_north.clear();
            way_pts_utm_east.clear();
            state = STATE_WAITING_FOR_WAYPOINTS;
            ROS_WARN("Switching to STATE_WAITING_FOR_WAYPOINTS");
        }

        else {
            ROS_ERROR("This should never happen.");
        }

    }

    return 0;
}


/* Publish rostopic on command line:

rostopic pub /rover_autonomy/coordinates rover_autonomy/gps_coord "{length: 1, latitudes:[49.901], longitudes:[8.901]}"

*/

