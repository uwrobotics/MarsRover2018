//UW Robotics - Mars Rover Team
//Arm Model Node for testing
//
//University of Waterloo
//Waterloo, Ontario, Canada, Earth
//Code Author: Kieran Ratcliffe
//Last updated: May 19, 2018
//Last updated by: Kieran Ratcliffe



#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float32MultiArray.h"
#include "can_msgs/Frame.h"
#include <math.h>

const int LOOP_PERIOD_MS = 10;
const int MS_PER_S = 1000; //Milliseconds per second
const int NUM_MOTORS = 5;
const float PI = 3.14159;
const float LINK_LENGTH[3] = {0.3,0.3,0.3};
const float BACK_EMF_CONSTANT[NUM_MOTORS] = {0.0163,0.0273,0.0485,0.0226,0.0354}; //rad/(sV), measured by giving motors 12V and timing how long it takes to make a certain angle

float inputPWM[NUM_MOTORS] = {0,0,0,0,0};
float qCurr[NUM_MOTORS] = {0,PI/4.0,-PI/2.0,-PI/1.1,0}; //Current position in radians
const int VOLTAGE = 12;
float xe, ye, ze, xw, yw, zw = 0; // Position of the end effector (e) and wrist (w) in the inertial frame

std_msgs::Float32MultiArray encoderMsg;

void positionUpdate();
void forwardKinematics();


void inputCallback(const std_msgs::Float32MultiArray& in_msg){
    encoderMsg.data.clear();

    std::vector<float> var;
    var = in_msg.data;
    float * tmp = var.data();

    for (int i = 0; i < NUM_MOTORS; i++) {
        inputPWM[i] = tmp[i];
    }
    positionUpdate();

    for (int i = 0; i < NUM_MOTORS; i++) {
    	encoderMsg.data.push_back(qCurr[i]);
    }

}

void positionUpdate() {
	for (int i = 0; i < NUM_MOTORS; i++) {
		qCurr[i] = qCurr[i] + VOLTAGE*inputPWM[i]*BACK_EMF_CONSTANT[i]*LOOP_PERIOD_MS/MS_PER_S;
		if (qCurr[i] >= PI) {
			qCurr[i] = -2.0*PI + qCurr[i];
		}
		else if (qCurr[i] <= -PI) {
			qCurr[i] = 2.0*PI + qCurr[i];
		}
	}
}


void forwardKinematics() {
	xe = cos(qCurr[0])*(LINK_LENGTH[0]*cos(qCurr[1]) + LINK_LENGTH[1]*cos(qCurr[1] + qCurr[2]) + LINK_LENGTH[2]*cos(qCurr[1] + qCurr[2] + qCurr[3]));
	ze = sin(qCurr[0])*(LINK_LENGTH[0]*cos(qCurr[1]) + LINK_LENGTH[1]*cos(qCurr[1] + qCurr[2]) + LINK_LENGTH[2]*cos(qCurr[1] + qCurr[2] + qCurr[3]));
	ye = (LINK_LENGTH[0]*sin(qCurr[1]) + LINK_LENGTH[1]*sin(qCurr[1] + qCurr[2]) + LINK_LENGTH[2]*sin(qCurr[1] + qCurr[2] + qCurr[3]));
	ROS_INFO("End-effector position is (xe: %f, ye: %f, ze %f) \n", xe, ye, ze);

	xw = cos(qCurr[0])*(LINK_LENGTH[0]*cos(qCurr[1]) + LINK_LENGTH[1]*cos(qCurr[1] + qCurr[2]));
	zw = sin(qCurr[0])*(LINK_LENGTH[0]*cos(qCurr[1]) + LINK_LENGTH[1]*cos(qCurr[1] + qCurr[2]));
	yw = (LINK_LENGTH[0]*sin(qCurr[1]) + LINK_LENGTH[1]*sin(qCurr[1] + qCurr[2]));
	ROS_INFO("Wrist position is (xw: %f, yw: %f, zw %f) \n", xw, yw, zw);
	
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "arm_model_node");
    ros::NodeHandle n;
    //dutyWriteArray.data.resize(NUM_DATA,0);
    
    ros::Publisher chatter_pub = n.advertise<std_msgs::Float32MultiArray>("model_output", 30);

    ros::Subscriber sub0 = n.subscribe("model_topic", 1, inputCallback);
  
    for (int i = 0; i < NUM_MOTORS; i++) {
    	encoderMsg.data.push_back(0.0);
    }


    ros::Rate loop_rate(LOOP_PERIOD_MS);
    int count = 0;
    int j = 0;
    while (ros::ok()) {
        ros::spinOnce();
        
        forwardKinematics();
        chatter_pub.publish(encoderMsg);

        loop_rate.sleep();
        ++count;
    }

  return 0;
}













