#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt32MultiArray.h>

ros::Subscriber encoderSub;
ros::Subscriber limitSwitchsub;


void encoderCallback(const std_msgs::UInt32MultiArray::ConstPtr& array)
{
	printf("Turntable: %d \n", array->data[0]);
	printf(" Shoulder: %d \n", array->data[1]);
	printf(" Elbow: %d \n", array->data[2]);
	printf(" Wrist P: %d \n", array->data[3]);
	printf(" Wrist R: %d \n" , array->data[4]);
	printf(" Claw: %d \n", array->data[5]);
	printf("\n\n\n");
	ros::Duration(1).sleep(); //To add delay for output. Duration in seconds.
}

void limitSwitchCallback(const std_msgs::UInt32MultiArray::ConstPtr& array)
{
	// printf("Turntable: %d \n", array->data[0]);
	// printf(" Shoulder: %d \n", array->data[1]);
	// printf(" Elbow: %d \n", array->data[2]);
	// printf(" Wrist P: %d \n", array->data[3]);
	// printf(" Wrist R: %d \n" , array->data[4]);
	// printf(" Claw: %d \n", array->data[5]);
	// printf("\n\n\n");
	// ros::Duration(1).sleep(); //To add delay for output. Duration in seconds.
	return;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "armtest"); //init the node

	ros::NodeHandle n;
	encoderSub = n.subscribe("encoders", 1, encoderCallback); 
	limitSwitchsub = n.subscribe("limitSwitches", 1, limitSwitchCallback);
	ros::spin();

	return 0;
}
