//  ///////////////////////////////////////////////////////////
//
// imu_test.cpp
// This File contains some random tests to see IMU works
// 
// Author: Chunshang Li. 2017 
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>
// #include <tf/LinearMath/Transform.h>

//Callback function for the Position topic 
void imu_callback(const sensor_msgs::Imu& msg)
{
    //This function is called when a new pose message is received

    tf::Quaternion q;
    tf::quaternionMsgToTF(msg.orientation, q);

    tf::Matrix3x3 rot_matrix(q);
    double roll, pitch, yaw;
    rot_matrix.getRPY(roll, pitch, yaw);
    geometry_msgs::Vector3 ang_vel = msg.angular_velocity;
    geometry_msgs::Vector3 lin_accel = msg.linear_acceleration;

    ROS_INFO("imu_callback RPY=(%9.6f %9.6f %9.6f), AngVel=(%9.6f %9.6f %9.6f), LinAccel=(%9.6f %9.6f %9.6f)", 
        roll*180/M_PI, pitch*180/M_PI, yaw*180/M_PI,
        ang_vel.x, ang_vel.y, ang_vel.z,
        lin_accel.x, lin_accel.y, lin_accel.z);
}

int main(int argc, char **argv)
{
    //Initialize the ROS framework
    ros::init(argc,argv,"imu_test");
    ros::NodeHandle n;

    // change log level
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber pose_sub = n.subscribe("/imu/data", 1, imu_callback);

    //Set the loop rate
    ros::Rate loop_rate(5);    //20Hz update rate
    
    while (ros::ok())
    {
        loop_rate.sleep(); //Maintain the loop rate
        ros::spinOnce();   //Check for new messages
    }

    return 0;
}
