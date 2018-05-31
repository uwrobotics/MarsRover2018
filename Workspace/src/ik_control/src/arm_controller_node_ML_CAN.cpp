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
#include <cmath>

////////////////////////////////
// Variable Declaration
////////////////////////////////

const float DUTY_SAFETY_FACTOR = 0.5; //Safety factor to ensure microcontroller not outputting full duty cycle (set to 1.0 if not desired)
const float JOYSTICK_DEADZONE = 0.12;
const float PI = 3.14159;
const int NUM_DATA = 6; //Number of pieces of data to send to the arduino
const int NUM_ARM_MOTORS = 5; //Number of motors that matter for IK purposes
const int CLAW_OPEN = 1.0; //Must either be 1 or -1, change so that pressing Right joystick, button 2 in claw mode OPENS the claw and button 3 CLOSES the claw
const int NUM_AXES_DATA = 5;
const int NUM_BUTTONS_DATA = 10;
const int NUM_BYTES_IN_FLOAT = 4; 
const int CAN_MOTOR_ID[NUM_DATA] = {400, 401, 402, 403, 404, 405}; //turntable, shoulder, elbow, wrist pitch, wrist roll, claw

//Xbox controller inputs
const int INDEX_A = 0; //sensor_msg.buttons index number for the left bumper
const int INDEX_B = 1; //sensor_msg.buttons index number for the right bumper
const int INDEX_LB = 4; //sensor_msg.buttons index number for the left bumper
const int INDEX_RB = 5; //sensor_msg.buttons index number for the right bumper
const int INDEX_BACK = 6; //sensor_msg.buttons index number for the back button
const int INDEX_LT = 2; //sensor_msg.axes index number for the left trigger
const int INDEX_RT = 5; //sensor_msg.axes index number for the right trigger
const int INDEX_LS_UD = 1; //sensor_msg.axes index number for the up/down value from the left joystick
const int INDEX_LS_LR = 0; //sensor_msg.axes index number for the right/left value from the left joystick
const int INDEX_RS_UD = 4; //sensor_msg.axes index number for the up/down value from the right joystick
const int INDEX_RS_LR = 3; //sensor_msg.axes index number for the right/left value from the right joystick


//Loop parameters
const int LOOP_PERIOD_MS = 10; //Maximum time for a control loop execution (that ignores new input) based on human reaction time

//Closed Loop Control Parameters
uint jointData[NUM_ARM_MOTORS] = {0,0,0,0,0}; //Angles of the joints
float jointDataFloat[NUM_ARM_MOTORS] = {0,0,0,0,0}; //Angles of the joints
const float ABS_ENCODER_TO_RAD = 2*PI/4098; //Conversion constant to go between absolute encoder values and radians
const float INC_ENCODER_TO_RAD = 2*PI/2048; //Conversion constant to go between incremental encoder values and radians (assume 2048 resolution)
const float ENCODER_TO_RAD[NUM_ARM_MOTORS] = {INC_ENCODER_TO_RAD,ABS_ENCODER_TO_RAD,ABS_ENCODER_TO_RAD,INC_ENCODER_TO_RAD,ABS_ENCODER_TO_RAD}; //Indicates the conversion factor for each motor and whether it is absolute or incremental
const float ENCODER_ZEROPOSITION[NUM_ARM_MOTORS] = {0,725,3550,0,940}; //The encoder value corresponding to its zero position
const int ENCODER_DIR[NUM_ARM_MOTORS] = {1,1,1,1,1}; //Determines if an increase in encoder values indicates an increase in assumed direction
const bool isRegClsLp[NUM_DATA] = {true,true,true,true,false,false}; //Is the motor regularly controlled via closed loop

//ML Control Parameters
float MAX_ARM_SPEED = 0.12; //Measured in m/s, semi-arbitrarily selected maximum Cartesian speed
float LINK_LENGTH[4] = {0.3,0.3,0.43};
const float BACK_EMF_CONSTANT[NUM_ARM_MOTORS] = {0.0163,0.0273,0.0485,0.0226,0.0354}; //rad/(sV), measured by giving motors 12V and timing how long it takes to make a certain angle
float weightJoints[NUM_ARM_MOTORS] = {1,1,1,1,1}; //Weight factors for joint angle PWM outputs that should tend the output towards the correct values
const float LEARNING_RATE[NUM_ARM_MOTORS] = {1.2,1.2,1.2,1.2,1.2}; //Learning rate for each joint
const float RUNTIME_FACTOR = 6000;


float outputPWM[NUM_DATA] = {0,0,0,0,0,0};
float qCurr[NUM_ARM_MOTORS] = {0,0,0,0,0}; //Current joint angles in radians
float qNext[NUM_ARM_MOTORS] = {0,0,0,0,0}; //Target joint angles in radians
const int VOLTAGE = 12;
float armStateInertial[NUM_ARM_MOTORS] = {0,0,0,0,0}; //State of the arm in intertial frame (theta, r/x, h/y, phi, psi)
//bool isWristAdjustMode = false; //Flag for detecting when the control mode is set to wrist adjust (indicating right joystick used to adjust wrist pitch instead of turntable movement)
float inputCommands[NUM_DATA] = {};


can_msgs::Frame motorCANFrames[NUM_DATA] = {};
std_msgs::Float32MultiArray modelOutputMsg;;
float axes[NUM_AXES_DATA] = {};
float buttons[NUM_BUTTONS_DATA] = {};

////////////////////////////////
// Function Declaration
////////////////////////////////

can_msgs::Frame populateFrame(can_msgs::Frame frameMsg, uint8_t bytesArray1[NUM_BYTES_IN_FLOAT], uint8_t bytesArray2[NUM_BYTES_IN_FLOAT]);
void frameFormerHelper();
void getMessageAxesData(const sensor_msgs::Joy& joy_msg, float arr[], int size);
void getMessageButtonsData(const sensor_msgs::Joy& joy_msg, float arr[], int size);
void populateInputCommands();
void forwardKinematics();
void calculateNextPosition();
void reverseModel();
void encoderConvert();

float solve_for_q0(float q[], float l[], float p[]);
float solve_for_q1(float q[], float l[], float p[]);
float solve_for_q2(float q[], float l[], float p[]);
float solve_for_q3(float q[], float l[], float p[]);
float solve_for_q4(float q[], float l[], float p[]);


////////////////////////////////
// Callback Definitions
////////////////////////////////

void joystickCallback(const sensor_msgs::Joy& joy_msg){

    getMessageAxesData(joy_msg,axes,NUM_AXES_DATA);
    getMessageButtonsData(joy_msg,buttons,NUM_BUTTONS_DATA);
    populateInputCommands();

}

void encoderCallback(const std_msgs::UInt32MultiArray& encoder_msg){
    std::vector<uint> var;
    var = encoder_msg.data;
    uint * tmp = var.data();

    for (int i = 0; i < NUM_ARM_MOTORS; i++) {
        jointData[i] = tmp[i];
    }

}

void modelCallback(const std_msgs::Float32MultiArray& model_msg){
    std::vector<float> var;
    var = model_msg.data;
    float * tmp = var.data();

    for (int i = 0; i < NUM_ARM_MOTORS; i++) {
        jointDataFloat[i] = tmp[i];
    }

}

////////////////////////////////
// Function Definition
////////////////////////////////

void encoderConvert() {
    for (int i = 0; i < NUM_ARM_MOTORS; i++) {
        //qCurr[i] = jointDataFloat[i];
        qCurr[i] = jointData[i]*ENCODER_TO_RAD[i] - ENCODER_ZEROPOSITION[i];
    }
}

void populateFrame(can_msgs::Frame *frameMsg, float val1) {
    memcpy(&frameMsg->data[0], (unsigned char*) (&val1), 4);
}

//In the case the operator chooses to take manual control of each individual joint
void populateInputCommands() {
    inputCommands[0] = (axes[INDEX_LT] - axes[INDEX_RT])/(-2.0);
    inputCommands[1] = axes[INDEX_LS_LR];
    inputCommands[2] = axes[INDEX_LS_UD];
    inputCommands[3] = axes[INDEX_RS_LR];
    inputCommands[4] = axes[INDEX_RS_UD];
    inputCommands[5] = 1.0*(buttons[0]-buttons[1]);

    for (int i = 0; i < NUM_DATA; i++) {
        if (fabs(inputCommands[i]) <= JOYSTICK_DEADZONE) {
            inputCommands[i] = 0.0;
        }
    }
}


void frameFormerHelper() {
    for (int i = 0; i < NUM_DATA; i++) {
        populateFrame(&motorCANFrames[i], DUTY_SAFETY_FACTOR*outputPWM[i]);
        motorCANFrames[i].id = CAN_MOTOR_ID[i];
        motorCANFrames[i].dlc = 4;
        motorCANFrames[i].is_error = 0;
        motorCANFrames[i].is_rtr = 0;
        motorCANFrames[i].is_extended = 0;
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

void forwardKinematics() {
    armStateInertial[1] = (LINK_LENGTH[0]*cos(qCurr[1]) + LINK_LENGTH[1]*cos(qCurr[1] + qCurr[2]));
    armStateInertial[2] = (LINK_LENGTH[0]*sin(qCurr[1]) + LINK_LENGTH[1]*sin(qCurr[1] + qCurr[2]));

    armStateInertial[0] = qCurr[0]; // Turntable
    armStateInertial[3] = qCurr[3] + qCurr[1] + qCurr[2]; // Wrist pitch
    armStateInertial[4] = qCurr[4]; // Wrist roll
}

void calculateNextPosition() {
    for (int j = 1; j <= 2; j++) {
        //ROS_INFO("Old arm position of motor %d and %f \n", j, armStateInertial[j]);
        armStateInertial[j] = armStateInertial[j] + MAX_ARM_SPEED*inputCommands[j]*LOOP_PERIOD_MS/1000;
        //ROS_INFO("New arm position of motor %d and %f \n", j, armStateInertial[j]);
    }
}

void reverseModel() {
    qNext[1] = solve_for_q1(qCurr, LINK_LENGTH, armStateInertial);
    qNext[2] = solve_for_q2(qCurr, LINK_LENGTH, armStateInertial);

    //ROS_INFO("qNext1 and qCurr1: %f and %f \n", qNext1, qCurr[1]);
    //ROS_INFO("qNext2 and qCurr2: %f and %f \n", qNext2, qCurr[2]);

    outputPWM[1] = (qNext[1] - qCurr[1])/(VOLTAGE*BACK_EMF_CONSTANT[1]*LOOP_PERIOD_MS/1000);
    outputPWM[2] = (qNext[2] - qCurr[2])/(VOLTAGE*BACK_EMF_CONSTANT[2]*LOOP_PERIOD_MS/1000);

    for (int j = 1; j <= 2; j++) {
        if (outputPWM[j] > 1) {
            outputPWM[j] = 1;
        }
        else if (outputPWM[j] < -1) {
            outputPWM[j] = -1;
        }
    }
}

//Weight update function for output learning
//w is the weight array meant to be updated via reference, t is the time increment, a is the learning rate
//ti is the target value at time t, and yt is the actual output at time t
float weightUpdate(float w, int t, float a, float Tt, float yt) {
    return w + (Tt-yt)*a*(RUNTIME_FACTOR/(t+RUNTIME_FACTOR));
}


////////////////////////////////
// Inverse Kinematics Functions
////////////////////////////////

//Solving for turntable
//q is the previous state of joints, l is link lengths, p is current position (r,h,theta,phi,psi)
float solve_for_q0(float q[], float l[], float p[]) {
    return p[0];
}

//Solving for shoulder
//q is the previous state of joints, l is link lengths, p is current position (r,h,theta,phi,psi)
float solve_for_q1(float q[], float l[], float p[]) {
    //p is the position 
    float x = p[1];
    float y = p[2];

    float r = sqrt(x*x + y*y);

    float a = ((l[0]*l[0])+r*r-(l[1]*l[1]))/(2*l[0]*r);
    float beta = 0;

    if (abs(a) >= 1) {
        //This if statement is to prevent errors due to being out of range of [-1,1]
        //This equation takes only the real part of the solution to acos(x) if abs(x)>1
        beta = atan2(sqrt(a*a-1),a);
    }
    else {
        beta = acos(a); //always gonna be between zero and pi
    }

    
    float gamma = atan2(y,x);
    //printf("ANGLES!: %f, %f", beta, gamma);
    float q1_1 = gamma - beta;
    float q1_2 = gamma + beta;

    if( abs(q[1]-q1_1) < abs(q[1]-q1_2) )
        return q1_1;
    else
        return q1_2;
}

//Solving for Elbow
//q is the previous state of joints, l is link lengths, p is current position (r,h,theta,phi,psi)
float solve_for_q2(float q[], float l[], float p[]) {
    //p is the position 
    float x = p[1];
    float y = p[2];

    float r = sqrt(x*x + y*y);
    float eta = acos( ((l[0]*l[0]) + (l[1]*l[1]) -r*r)/(2*(l[0]*l[1])) ); //always gonna be between zero and pi
    float q2_1 = PI-eta;
    float q2_2 = eta - PI;

    if( abs(q[2]-q2_1) < abs(q[2]-q2_2)) {
      return q2_1;
    }
    else {
      return q2_2;
    }
}

//Solving for Wrist Pitch
//q is the previous state of joints, l is link lengths, p is current position (r,h,theta,phi,psi)
//This assumes set wrist pitch angle
//The current method is a work-around to avoid segfaults
float solve_for_q3(float q[], float l[], float p[]){
    float q1_tmp = solve_for_q1(qCurr, LINK_LENGTH, armStateInertial);
    float q2_tmp = solve_for_q2(qCurr, LINK_LENGTH, armStateInertial);

    return p[3]-q1_tmp-q2_tmp;
}

//Solve for wrist roll
//q is the previous state of joints, l is link lengths, p is current position (r,h,theta,phi,psi)
float solve_for_q4(float q[], float l[], float p[]) {
    return p[4];
}


////////////////////////////////
// Main Loop
////////////////////////////////


int main(int argc, char **argv) {
    ros::init(argc, argv, "arm_controller_node_ML");
    ros::NodeHandle n;
    //dutyWriteArray.data.resize(NUM_DATA,0);
    
    ros::Publisher chatter_pub = n.advertise<can_msgs::Frame>("/CAN_transmitter", 30);
    ros::Publisher chatter_pub1 = n.advertise<std_msgs::Float32MultiArray>("model_topic", 30);

    ros::Subscriber sub0 = n.subscribe("/encoders", 1, encoderCallback);
    ros::Subscriber sub2 = n.subscribe("/joy", 1, joystickCallback);
    ros::Subscriber sub3 = n.subscribe("model_output", 1, modelCallback);

    for (int i = 0; i < NUM_ARM_MOTORS; i++) {
        modelOutputMsg.data.push_back(0.0);
    }  


    ros::Rate loop_rate(LOOP_PERIOD_MS);
    int count = 0;
    int t = 1;
    while (ros::ok()) {
        ros::spinOnce();
        //ROS_INFO("Count: %d \n", count);


        //Calculate the outputPWM
        encoderConvert();
        outputPWM[5] = inputCommands[5];

        for (int i = 1; i <= 3; i++) {
            weightJoints[i] = weightUpdate(weightJoints[i], t, LEARNING_RATE[i], qNext[i], qCurr[i]);
        }
        if ((buttons[INDEX_LB] == 1) && (buttons[INDEX_RB] == 1)) {
            //IK control scheme
            outputPWM[0] = inputCommands[0];
            outputPWM[4] = inputCommands[4];
            
            forwardKinematics();
            calculateNextPosition();
            reverseModel();


            if (buttons[INDEX_BACK] == 1) { //Wrist pitch set mode (i.e. adjust relative angle of wirst pitch to the ground otherwise remains constant)
                outputPWM[3] = inputCommands[3];
            }
            else { //Wrist pitch stays the same
                qNext[3] = solve_for_q3(qCurr, LINK_LENGTH, armStateInertial);
                outputPWM[3] = (qNext[3] - qCurr[3])/(VOLTAGE*BACK_EMF_CONSTANT[3]*LOOP_PERIOD_MS/1000);
                if (outputPWM[3] > 1) {
                    outputPWM[3] = 1;
                }
                else if (outputPWM[3] < -1) {
                    outputPWM[3] = -1;
                }
            }

        }
        else  { //Manual Control Scheme
            for (int i = 0; i < NUM_DATA; i++) {
                outputPWM[i] = inputCommands[i];
            }

        }

        modelOutputMsg.data.clear();
        for (int i = 0; i < NUM_ARM_MOTORS; i++) {
            if (fabs(outputPWM[i]) >= JOYSTICK_DEADZONE) {
                //ROS_INFO("Position of motor %d is: %f \n", i, outputPWM[i]);
                modelOutputMsg.data.push_back(outputPWM[i]*DUTY_SAFETY_FACTOR);
            }
            else {
                modelOutputMsg.data.push_back(0.0);
            }
            
            //ROS_INFO("Output PWM of motor %d is: %f \n", i, outputPWM[i]);
        }
        //Done calculating outputPWM

        for (int i = 0; i < NUM_ARM_MOTORS; i++) {
            //ROS_INFO("Output PWM of motor %d is: %f \n", i, outputPWM[i]);
        }


        //chatter_pub1.publish(modelOutputMsg);
        

        frameFormerHelper();
        for (int i = 0; i < NUM_DATA; i++) {
            chatter_pub.publish(motorCANFrames[i]);
        }

        loop_rate.sleep();
        ++count;
        ++t;
    }

  return 0;
}



