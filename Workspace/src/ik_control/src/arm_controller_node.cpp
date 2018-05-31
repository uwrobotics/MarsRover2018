//UW Robotics - Mars Rover Team
//Arm Control PID Executable
//
//University of Waterloo
//Waterloo, Ontario, Canada, Earth
//Code Author: Kieran Ratcliffe
//Last updated: May 19, 2018
//Last updated by: Kieran Ratcliffe


#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/UInt32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "can_msgs/Frame.h"

////////////////////////////////
// Variable Declaration
////////////////////////////////

const float DUTY_SAFETY_FACTOR = 0.5; //Safety factor to ensure microcontroller not outputting full duty cycle (set to 1.0 if not desired)
const int JOYSTICK_DEADZONE = 0;
const int NUM_DATA = 6; //Number of pieces of data to send to the arduino
const int CLAW_OPEN = 1.0; //Must either be 1 or -1, change so that pressing Right joystick, button 2 in claw mode OPENS the claw and button 3 CLOSES the claw
const int NUM_AXES_DATA = 3;
const int NUM_BUTTONS_DATA = 10;
const int NUM_BYTES_IN_FLOAT = 4;
const int CAN_MOTOR_ID[NUM_DATA] = {400, 401, 402, 403, 404, 405}; //turntable, shoulder, elbow, wrist pitch, wrist roll, claw

//Xbox controller inputs
const int INDEX_A = 0; //sensor_msg.buttons index number for the left bumper
const int INDEX_B = 1; //sensor_msg.buttons index number for the right bumper
const int INDEX_LB = 4; //sensor_msg.buttons index number for the left bumper
const int INDEX_RB = 5; //sensor_msg.buttons index number for the right bumper
const int INDEX_LT = 2; //sensor_msg.axes index number for the left trigger
const int INDEX_RT = 5; //sensor_msg.axes index number for the right trigger
const int INDEX_LS_UD = 1; //sensor_msg.axes index number for the up/down value from the left joystick
const int INDEX_LS_LR = 0; //sensor_msg.axes index number for the right/left value from the left joystick
const int INDEX_RS_UD = 4; //sensor_msg.axes index number for the up/down value from the right joystick
const int INDEX_RS_LR = 3; //sensor_msg.axes index number for the right/left value from the right joystick


//Loop parameters
const int LOOP_LENGTH_MS = 100; //Maximum time for a control loop execution (that ignores new input) based on human reaction time
const int LOOP_PERIOD_MS = 5; //Minimum period based on maximum encoder period
const int NUM_SETPOINT_DATA = ceil(LOOP_LENGTH_MS/LOOP_PERIOD_MS);
bool isNewData = false;
bool isInStopMode = false; //Should the arm stop moving, if so output 0 pwm to the motors

//Control Parameters
float setpointData[NUM_DATA][NUM_SETPOINT_DATA] = {};
uint stateData[NUM_DATA] = {}; //Angles of the joints
const float ABS_ENCODER_TO_RAD = 2*3.14159/4098; //Conversion constant to go between absolute encoder values and radians
const float INC_ENCODER_TO_RAD = 2*3.14159/2048; //Conversion constant to go between incremental encoder values and radians (assume 2048 resolution)
const float ENCODER_ZEROPOSITION[NUM_DATA] = {0,725,3550,0,940,0}; //The encoder value corresponding to its zero position
const float PID_GAINS[NUM_DATA][3] = { //Order is Kp, Kd, Ki
    {5,0.45,0.01}, //Gain for the turntable
    {5,0.45,0.01}, //Gain for the shoulder
    {5,0.45,0.01}, //Gain for the elbow
    {5,0.45,0.01}, //Gain for the wrist pitch
    {1,0,0}, //Gain for wrist roll
    {1,0,0} //Gain for claw
};
float stateErrorCurr[NUM_DATA] = {0,0,0,0,0,0}; //Error for the current iteration of the control loop
float stateErrorPrev1[NUM_DATA] = {0,0,0,0,0,0}; //Error for the previous iteration of the control loop (initalize with zero if new loop)
float stateErrorPrev2[NUM_DATA] = {0,0,0,0,0,0}; //Error for the second most previous iteration of the control loop (initalize with zero if new loop)
float stateErrorSum[NUM_DATA] = {0,0,0,0,0,0}; //Total error over time
const bool isRegClsLp[NUM_DATA] = {true,true,true,true,false,false}; //Is the motor regularly controlled via closed loop
const int MAX_VOLTAGE = 12; //Positive and negative saturation point
bool isClosedLoopControl = true; //A flag to indicate when to used closed loop mode versus open loop control (i.e control each motor separately)
const float CLOSED_LOOP_PERIOD_S = 0.005; //The period of the closed loop in seconds (it is a float in case we want to change to 4.5 for example)

can_msgs::Frame motorCANFrames[NUM_DATA] = {};
float axes[NUM_AXES_DATA] = {};
float buttons[NUM_BUTTONS_DATA] = {};
float outputPWM[NUM_DATA] = {}; //Order of the motors is turntable, shoulder, elbow, wrist pitch, wrist roll, claw
float outputPWMPrev1[NUM_DATA] = {}; //Order of the motors is turntable, shoulder, elbow, wrist pitch, wrist roll, claw
float outputPWMPrev2[NUM_DATA] = {}; //Order of the motors is turntable, shoulder, elbow, wrist pitch, wrist roll, claw

////////////////////////////////
// Function Declaration
////////////////////////////////

void errorUpdate(int i);
can_msgs::Frame populateFrame(can_msgs::Frame frameMsg, uint8_t bytesArray1[NUM_BYTES_IN_FLOAT], uint8_t bytesArray2[NUM_BYTES_IN_FLOAT]);
void frameFormerHelper();
void getMessageAxesData(const sensor_msgs::Joy& joy_msg, float arr[], int size);
void getMessageButtonsData(const sensor_msgs::Joy& joy_msg, float arr[], int size);
void opneLoopOutput(float axes[], float buttons[]);
float closedLoopPIDOutputZ(const float eCurr, const float ePrev1, const float ePrev2, const float uPrev2, float gainsPID[]);
float closedLoopPIDOutputS(const float eCurr, const float ePrev, const float eSum, float gainsPID[]);

////////////////////////////////
// Callback Definitions
////////////////////////////////

void joystickCallback(const sensor_msgs::Joy& joy_msg){
    getMessageAxesData(joy_msg,axes,NUM_AXES_DATA);
    getMessageButtonsData(joy_msg,buttons,NUM_BUTTONS_DATA);
    
    isClosedLoopControl = true;

    if ((buttons[INDEX_LB] == 1) && (buttons[INDEX_RB] == 1)) {
        isClosedLoopControl = false;
        isInStopMode = false;
    }

}

void encoderCallback(const std_msgs::UInt32MultiArray& encoder_msg){
    std::vector<uint> var;
    var = encoder_msg.data;
    uint * tmp = var.data();

    for (int i = 0; i < NUM_DATA; i++) {
        stateData[i] = tmp[i];
    }

}

void IKCallback(const std_msgs::Float32MultiArray& setpoint_msg) {
    if (setpoint_msg.data.size() == NUM_DATA*NUM_SETPOINT_DATA) { //If not the estimated size
        std::vector<float> var;
        var = setpoint_msg.data;
        float * tmp = var.data();
        isInStopMode = false;
        for (int i = 0; i < NUM_DATA; i++) {
            stateErrorPrev1[i] = 0;
            stateErrorCurr[i] = 0;
            stateErrorSum[i] = 0;
        }

        for (int i = 0; i < NUM_DATA; i++) {
            for (int j = 0; j < NUM_SETPOINT_DATA; j++) {
                setpointData[i][j] = tmp[NUM_SETPOINT_DATA*i+j];
                ROS_INFO("setpoint_msg value is %f \n",tmp[NUM_SETPOINT_DATA*i+j]);
            }
        }
        ROS_INFO("Done sending data \n");

        isInStopMode = false;
        isNewData = true;
    }
    else {
        isInStopMode = true;
    }

}

////////////////////////////////
// Function Definition
////////////////////////////////

void encoderConvert() {

}

void populateFrame(can_msgs::Frame *frameMsg, float val1) {
    memcpy(&frameMsg->data[0], (unsigned char*) (&val1), 4);
}

//In the case the operator chooses to take manual control of each individual joint
void opneLoopOutput(float axes[], float buttons[]) {
    outputPWM[0] = (axes[INDEX_LT] - axes[INDEX_RT])/(-2.0);
    outputPWM[1] = axes[INDEX_LS_LR];
    outputPWM[2] = axes[INDEX_LS_UD];
    outputPWM[3] = axes[INDEX_RS_LR];
    outputPWM[4] = axes[INDEX_RS_UD];
    outputPWM[5] = (buttons[0]-buttons[1]);
}

//This function runs one iteration of the PWM closed loop control for one motor (use discrete controls domain with s->z transformations)
//Control effort is measured in volts then divided by MAX_VOLTAGE for PWM signal
float closedLoopPIDOutputZ(const float eCurr, const float ePrev1, const float ePrev2, const float uPrev2, float gainsPID[]){
    //TODO: Confirm this logic

    float e0 = eCurr*(gainsPID[0] + 2.0*gainsPID[1]/CLOSED_LOOP_PERIOD_S + gainsPID[2]*CLOSED_LOOP_PERIOD_S/2.0);
    float e1 = ePrev1*(-4.0*gainsPID[1]/CLOSED_LOOP_PERIOD_S + gainsPID[2]*CLOSED_LOOP_PERIOD_S);
    float e2 = ePrev2*(-gainsPID[0] + 2.0*gainsPID[1]/CLOSED_LOOP_PERIOD_S + gainsPID[2]*CLOSED_LOOP_PERIOD_S/2.0);
    float u2 = -uPrev2;
    float u = e0 + e1 + e2 - u2;

    if (u >= MAX_VOLTAGE) {
        u = MAX_VOLTAGE;
    }
    else if (u <= -1.0*MAX_VOLTAGE) {
        u = -MAX_VOLTAGE;
    }

    return u/MAX_VOLTAGE;
}

//This function runs one iteration of the PWM closed loop control for one motor (use literal functions of PID gains)
//Control effort is measured in volts then divided by MAX_VOLTAGE for PWM signal
float closedLoopPIDOutputS(const float eCurr, const float ePrev, const float eSum, float gainsPID[]){
    //TODO: Confirm this logic

    float P = gainsPID[0]*eCurr;
    float D = gainsPID[1]*(eCurr - ePrev)/CLOSED_LOOP_PERIOD_S;
    float I = gainsPID[2]*eSum;
    float u = P + I + D;

    if (u >= MAX_VOLTAGE) {
        u = MAX_VOLTAGE;
    }
    else if (u <= -1.0*MAX_VOLTAGE) {
        u = -MAX_VOLTAGE;
    }

    return u/MAX_VOLTAGE;
}


void frameFormerHelper() {
    for (int i = 0; i < NUM_DATA; i++) {
        populateFrame(&motorCANFrames[i], DUTY_SAFETY_FACTOR*outputPWM[i]);
        motorCANFrames[i].id = CAN_MOTOR_ID[i]+i;
        motorCANFrames[i].dlc = 4;
        motorCANFrames[i].is_error = 0;
        motorCANFrames[i].is_rtr = 0;
        motorCANFrames[i].is_extended = 0;
    }
}

//Shift the error down the line (i.e. last previous error becomes second last previous error)
//Calculated the current error based on current state and setpoint data
//Shift output effort similarly to error
void errorUpdate(float set[]) {
    //Previous error should be stored in stateErrorPrev1
    for(int k = 0; k < NUM_DATA; k++) {
        stateErrorPrev2[k] = stateErrorPrev1[k];
        stateErrorPrev1[k] = stateErrorCurr[k];
        stateErrorCurr[k] = set[k] - stateData[k];

        stateErrorSum[k] = stateErrorSum[k] + (stateErrorCurr[k] + stateErrorPrev1[k])*CLOSED_LOOP_PERIOD_S/2;

        outputPWMPrev2[k] = outputPWMPrev1[k];
        outputPWMPrev1[k] = outputPWM[k];
    }
}

//Set the values of the global array to the values in the joy_msg
void getMessageAxesData(const sensor_msgs::Joy& joy_msg, float arr[], int size) {
    for (int i=0; i <= size; i++) {
        arr[i] = joy_msg.axes[i];
    }
}

//Set the values of the global array to the values in the joy_msg
void getMessageButtonsData(const sensor_msgs::Joy& joy_msg, float arr[], int size) {
    for (int i=0; i <= size; i++) {
        arr[i] = joy_msg.buttons[i];
    }
}

////////////////////////////////
// Main Loop
////////////////////////////////


int main(int argc, char **argv) {
    ros::init(argc, argv, "arm_controller_node");
    ros::NodeHandle n;
    //dutyWriteArray.data.resize(NUM_DATA,0);
    
    ros::Publisher chatter_pub = n.advertise<can_msgs::Frame>("/CAN_transmitter", 30);

    ros::Subscriber sub0 = n.subscribe("/encoders", 1, encoderCallback);
    ros::Subscriber sub1 = n.subscribe("IK_topic", 1, IKCallback);
    ros::Subscriber sub2 = n.subscribe("/joy0/joy", 1, joystickCallback);
  
    ros::Rate loop_rate(10);
    int count = 0;
    int j = 0;
    while (ros::ok()) {
        ros::spinOnce();

        if (isNewData) {
            j = 0;
        }

        if (isClosedLoopControl) {
            //Closed Loop Code
            if ((j <= NUM_SETPOINT_DATA)) {
                float setPointInstance[NUM_DATA] = {};
                for (int i = 0; i < NUM_DATA; i++) {
                    setPointInstance[i] = setpointData[i][j];
                }
                errorUpdate(setPointInstance);

                for (int i = 0; i < NUM_DATA; i++) {
                    outputPWM[i] = 0;
                }

                for (int i = 0; i < NUM_DATA; i++) {
                    float pid[3] = {PID_GAINS[i][0],PID_GAINS[i][1],PID_GAINS[i][2]};
                    outputPWM[i] = closedLoopPIDOutputZ(stateErrorCurr[i],stateErrorPrev1[i],stateErrorPrev2[i],stateErrorSum[i],pid);
                }
                j++;
            }
            else {
                isInStopMode = true;
            }
        }
        else { 
            //Open Loop Code
            opneLoopOutput(axes, buttons);
        }

        if (isInStopMode) {
            for (int i = 0; i < NUM_DATA; i++) {
                outputPWM[i] = 0;
            }
        }
        

        frameFormerHelper();
        for (int i = 0; i < NUM_DATA; i++) {
            chatter_pub.publish(motorCANFrames[i]);
        }

        isNewData = false;
        loop_rate.sleep();
        ++count;
    }

  return 0;
}



