#ifndef IK_h
#define IK_h

#include "ckp_Matrix.h"
#include <cmath>
const float PI = 3.14159;
float LINK_LENGTH[4] = {0.3,0.3,0.3,0.43};
const int NUM_ARM_MOTORS = 5;
const float BACK_EMF_CONSTANT[NUM_ARM_MOTORS] = {0.0163,0.0273,0.0485,0.0226,0.0354}; //rad/(sV), measured by giving motors 12V and timing how long it takes to make a certain angle


//Solving for turntable
float solve_for_q0(float q[], float l[], float p[]);

//Solving for shoulder
float solve_for_q1(float q[], float l[], float p[]);

//Solving for elbow
float solve_for_q2(float q[], float l[], float p[]);

//Solve for constant wrist pitch
float solve_for_q3(float q[], float l[], float p[]);

//Solve for wrist roll
float solve_for_q4(float q[], float l[], float p[]);


#endif

