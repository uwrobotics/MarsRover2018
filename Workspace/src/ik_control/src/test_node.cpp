//UW Robotics - Mars Rover Team
//Arm Control PID Executable
//
//University of Waterloo
//Waterloo, Ontario, Canada, Earth
//Code Author: Kieran Ratcliffe
//Last updated: May 19, 2018
//Last updated by: Kieran Ratcliffe


#include "ros/ros.h"
//#include "sensor_msgs/Joy.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/UInt32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "can_msgs/Frame.h"


const float DUTY_SAFETY_FACTOR = 0.5; //Safety factor to ensure microcontroller not outputting full duty cycle (set to 1.0 if not desired)
const int JOYSTICK_DEADZONE = 0;
const int NUM_DATA = 6; //Number of pieces of data to send to the arduino
const int CLAW_OPEN = 1.0; //Must either be 1 or -1, change so that pressing Right joystick, button 2 in claw mode OPENS the claw and button 3 CLOSES the claw
const int NUM_AXES_DATA = 3;
const int NUM_BUTTONS_DATA = 10;
const int NUM_BYTES_IN_FLOAT = 4;
const int CAN_MOTOR_ID[NUM_DATA] = {400, 401, 402, 403, 404, 405}; //turntable, shoulder, elbow, wrist pitch, wrist roll, claw

//Loop parameters
const int LOOP_LENGTH_MS = 100; //Maximum time for a control loop execution (that ignores new input) based on human reaction time
const int LOOP_PERIOD_MS = 5; //Minimum period based on maximum encoder period
const int NUM_SETPOINT_DATA = ceil(LOOP_LENGTH_MS/LOOP_PERIOD_MS);
bool isNewData = false;
bool isInStopMode = false; //Should the arm stop moving, if so output 0 pwm to the motors

//Control Parameters
float setpointData[NUM_DATA][NUM_SETPOINT_DATA] = {};
uint stateData[NUM_DATA] = {}; //Angles of the joints
const float PID_GAINS[NUM_DATA][3] = {
    {5,0.45,0.01}, //Gain for the turntable
    {5,0.45,0.01}, //Gain for the shoulder
    {5,0.45,0.01}, //Gain for the elbow
    {5,0.45,0.01}, //Gain for the wrist pitch
    {1,0,0}, //Gain for wrist roll
    {1,0,0} //Gain for claw
};
float stateErrorCurr[NUM_DATA] = {0,0,0,0,0,0}; //Error for the current iteration of the control loop
float stateErrorPrev[NUM_DATA] = {0,0,0,0,0,0}; //Error for the previous iteration of the control loop (initalize with zero if new loop)
float stateErrorSum[NUM_DATA] = {0,0,0,0,0,0}; //Total error over time
bool isClosedLoop[NUM_DATA] = {false,true,true,true,false,false};
const int MAX_VOLTAGE = 12; //Positive and negative saturation point

can_msgs::Frame motorCANFrames[NUM_DATA] = {};


float dutyWriteClaw = 0;
float dutyWriteWristRoll = 0;
float dutyWriteWristPitch = 0;
float dutyWriteElbow = 0;
float dutyWriteShoulder = 0;
float dutyWriteTurntable = 0;

float axesRight[NUM_AXES_DATA] = {};
float axesLeft[NUM_AXES_DATA] = {};
float buttonsRight[NUM_BUTTONS_DATA] = {};
float buttonsLeft[NUM_BUTTONS_DATA] = {};
float outputPWM[NUM_DATA] = {};

void getCurrErr(int i);
can_msgs::Frame populateFrame(can_msgs::Frame frameMsg, uint8_t bytesArray1[NUM_BYTES_IN_FLOAT], uint8_t bytesArray2[NUM_BYTES_IN_FLOAT]);
void frameFormerHelper();

void canCallback(const can_msgs::Frame& motor_msg){

}

void populateFrame(can_msgs::Frame *frameMsg, float val1) {
    memcpy(&frameMsg->data[0], (unsigned char*) (&val1), 4);
}


void frameFormerHelper() {
    for (int i = 0; i < NUM_DATA; i++) {
        populateFrame(&motorCANFrames[i], outputPWM[i]);
        motorCANFrames[i].id = CAN_MOTOR_ID[i]+i;
        motorCANFrames[i].dlc = 4;
        motorCANFrames[i].is_error = 0;
        motorCANFrames[i].is_rtr = 0;
        motorCANFrames[i].is_extended = 0;
    }
}

void getCurrErr(int i) {
    //Previous error should be stored in stateErrorPrev
    for(int k = 0; k < NUM_DATA; k++) {
        stateErrorCurr[k] = setpointData[k][i] - stateData[k];
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "test_node");
    ros::NodeHandle n;
    //dutyWriteArray.data.resize(NUM_DATA,0);
    
    ros::Publisher chatter_pub1 = n.advertise<std_msgs::UInt32MultiArray>("encoders", 30);
    ros::Publisher chatter_pub2 = n.advertise<std_msgs::Float32MultiArray>("IK_topic", 30);

    ros::Subscriber sub0 = n.subscribe("/CAN_transmitter", 1, canCallback);
  	
    uint encoderValuesTmp[NUM_DATA] = {1200,2300,3400,4500,5600,6700};
    std_msgs::UInt32MultiArray encoderValues;
	for (int i = 0; i < NUM_DATA; i++) {
		encoderValues.data.push_back(encoderValuesTmp[i]);
	}

    std_msgs::Float32MultiArray setpointValues;

    for (int i = 0; i < NUM_DATA; i++) {
        for (int j = 0; j < NUM_SETPOINT_DATA; j++) {
            setpointValues.data.push_back(900.0*(i+1) +200.0*cos(1.5*j/2.5));
            //ROS_INFO("setpoint_msg value is %f \n",tmp[NUM_SETPOINT_DATA*i+j]);
        }
    }


    ros::Rate loop_rate(10);
    int count = 0;
    while (ros::ok()) {
        ros::spinOnce();
        
        chatter_pub1.publish(encoderValues);
        chatter_pub2.publish(setpointValues);

        
        loop_rate.sleep();
        ++count;
    }

  return 0;
}

