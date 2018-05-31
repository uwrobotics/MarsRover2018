#include "ckp_Matrix.h"
#include "IK.h"
#include <cmath>

using namespace std;
//The state of the arm is defined in a cylindrical coordinate frame which is controlled by turntable, shoulder, and elbow (theta 0-2 respectively)
//Furthermore, the orientation of the end effector is defined by phi (angle of end effector to the groun) and psi (roll of the end effector), controlled by wrist pitch and wrist roll respectively (theta 4 and theta 5 respectively)


//Solving for turntable
//q is the previous state of joints, l is link lengths, p is current position (r,h,theta,phi,psi)
float solve_for_q0(float q[], float l[], float p[]) {
    return p[2];
}

//Solving for shoulder
//q is the previous state of joints, l is link lengths, p is current position (r,h,theta,phi,psi)
float solve_for_q1(float q[], float l[], float p[]) {
    //p is the position 
    float x = p[0];
    float y = p[1];

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
    float x = p[0];
    float y = p[1];

    float r = sqrt(x*x + y*y);
    float eta = acos( ((l[0]*l[0]) + (l[1]*l[1]) -r*r)/(2*(l[0]*l[1])) ); //always gonna be between zero and pi
    float q2_1 = pi-eta;
    float q2_2 = eta - pi;

    if( abs(q[2]-q2_1) < abs(q[2]-q2_2)) {
      return q2_1;
    }
    else {
      return q2_2;
    }
}

//Solving for Wrist Pitch
//q is the previous state of joints, l is link lengths, p is current position (r,h,theta,phi,psi)
//This assumes constant wrist pitch angle
float solve_for_q3(float q[], float l[], float p[]){
    return p[3]-solve_for_q2(q, l, p)-solve_for_q3(q, l, p);
}

//Solve for wrist roll
//q is the previous state of joints, l is link lengths, p is current position (r,h,theta,phi,psi)
float solve_for_q4(float q[], float l[], float p[]) {
    return p[4];
}


