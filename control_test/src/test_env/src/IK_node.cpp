#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/UInt32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "IK.h"
#include "ckp_Matrix.h"
// #include "spline.h"

using namespace std;
/////////////////////////////////////////////////
/////////////Robot Parameters////////////////////
/////////////////////////////////////////////////
const float bicep = 0.3048; //m
const float forearm = 0.3048; //m
const float wrist = 0; //m
const float limit = bicep+forearm-0.01;

const int links = 3;

//Number of angles
const int dof = 6;

//The link dimensions
Matrix a = Matrix(links,1);



Matrix alpha = Matrix(dof,1);
Matrix d = Matrix(dof,1);
Matrix q_i = Matrix(dof,1); //initial
Matrix q_f = Matrix(dof,1); //final
//q(0,0) = Turntable
//q(1,0) = Shoulder
//q(2,0) = Elbow
//q(3,0) = Wrist_Pitch
//q(4,0) = Wrist_Roll
//q(5,0) = Claw

Matrix p_i = Matrix(3,1); //initial
Matrix p_f = Matrix(3,1); //final
//p(0,0) = x position of end effector
//p(1,0) = y position of end effector
//p(2,0) = psi position of end effector



////////////////////////////////////////////////////
/////////////Controls Parameters////////////////////
////////////////////////////////////////////////////
std_msgs::Float32MultiArray angleMsg;
bool is_q3_const = 1; //button PRESSED sets this to 0.
const float V_MAX = 0.05; // m/s
const float W_MAX = pi/6; //rad/s

//Update angles every time_exe seconds
const float time_exe = 0.1; //Seconds
const int n_points = 20;
float dt = time_exe/(n_points);
Matrix v = Matrix(dof,1);
//v(0,0) = Angular velocity of q0
//v(1,0) = Linear velocity of x
//v(2,0) = Linear velocity of y
//v(3,0) = Angular velocity of q3
//v(4,0) = Angular velocity of q4
//v(5,0) = Angular velocity of q5


//The array for
Matrix q0_arr = Matrix(n_points, 1);
Matrix q1_arr = Matrix(n_points, 1);
Matrix q2_arr = Matrix(n_points, 1);
Matrix q3_arr = Matrix(n_points, 1);
Matrix q4_arr = Matrix(n_points, 1);
Matrix q5_arr = Matrix(n_points, 1);

/////////////////////////////////////////////////
/////////////Helper Functions////////////////////
/////////////////////////////////////////////////

//Set the values of the global array to the values in the joy_msg
void updateVelocity(const sensor_msgs::Joy& joy_msg);
void getControlScheme(const sensor_msgs::Joy& joy_msg);
void threshold();
void updatePosition();
void update_angleMsg();
void MainLoop();



/////////////////////////////////////////////////////
/////////////Subscriber Functions////////////////////
/////////////////////////////////////////////////////
void joyCallback(const sensor_msgs::Joy& joy_msg){
    //must implement control scheme
    getControlScheme(joy_msg);
    updateVelocity(joy_msg);
    threshold();
    printf("w_tt: %f \n", v(0,0));
    printf("v_x: %f \n", v(1,0));
    printf("v_y: %f \n", v(2,0));
    printf("w_wp: %f \n", v(3,0));
    printf("w_wr: %f \n", v(4,0));
    printf("w_claw : %f \n", v(5,0));
    printf("\n\n\n");
    MainLoop();
}

//Encoder Callback functions
//To be completed when the encoder is connected
void currentAngle(const std_msgs::UInt32MultiArray::ConstPtr& encoder){
    for (int i =0; i<dof; i++)
    {
        q_i(i,0) = encoder->data[i];
    }
    //Angle Conversion???
    q0_arr(0,0) = q_i(0,0);
    q1_arr(0,0) = q_i(1,0);
    q2_arr(0,0) = q_i(2,0);
    q3_arr(0,0) = q_i(3,0);
    q4_arr(0,0) = q_i(4,0);
    q5_arr(0,0) = q_i(5,0);
}


void test_print_before()
{
  printf("BEFORE: \n\n");
  printf("x_i %f \n", p_i(0,0));
  printf("y_i %f \n", p_i(1,0));
  printf("Psi_i %f \n", p_i(2,0));
  printf("Turntable %f \n", q_i(0,0));
  printf("Shoulder %f \n", q_i(1,0));
  printf("Elbow %f \n", q_i(2,0));
  printf("Wrist Pitch %f \n", q_i(3,0));
  printf("Wrist Roll %f \n", q_i(4,0));
  printf("Claw %f \n", q_i(5,0));
  printf("\n\n");
}

void test_print_after()
{
  printf("AFTER: \n\n");
  printf("x_f %f \n", p_i(0,0));
  printf("y_f %f \n", p_i(1,0));
  printf("Psi_f %f \n", p_i(2,0));
  printf("Turntable %f \n", q0_arr(19,0));
  printf("Shoulder %f \n", q1_arr(19,0));
  printf("Elbow %f \n", q2_arr(19,0));
  printf("Wrist Pitch %f \n", q3_arr(19,0));
  printf("Wrist Roll %f \n", q4_arr(19,0));
  printf("Claw %f \n", q5_arr(19,0));
  printf("\n\n");
}

void test_print_msg()
{
  printf("\n\n Start: ");
  for(int i = 0; i<(4*n_points); i++)
    printf("%f, ", angleMsg.data[i]);
  printf("\n\n");
}

int main(int argc, char **argv) {
    a(0,0) = bicep; //0.3048
    a(1,0) = forearm;
    a(2,0) = wrist;
    q_i(1,0) = pi/3;
    q_i(2,0) = pi/6;
    p_i(2,0) = q_i(1,0) + q_i(2,0) +q_i(3,0);
    ros::init(argc, argv, "IK_node");
    ros::NodeHandle n;

    ros::Publisher PID_pub = n.advertise<std_msgs::Float32MultiArray>("IK_topic", 1);

    ros::Subscriber sub0 = n.subscribe("/joy", 1, joyCallback);
    //Uncomment for later. Just commented out for testing
    ros::Subscriber encoderSub = n.subscribe("/encoders",1,currentAngle);


    ros::Rate loop_rate(10);
    int count = 0;
    while (ros::ok()) {
      ros::spinOnce();
	    PID_pub.publish(angleMsg);
      loop_rate.sleep();
      ++count;
    }
  return 0;
}

/////////////////////////////////////////////////
/////////////Helper Functions////////////////////
/////////////////////////////////////////////////

//Set the values of the global array to the values in the joy_msg
void updateVelocity(const sensor_msgs::Joy& joy_msg)
{
    //v(0,0) = Angular velocity of q0 (turntable)
    //v(1,0) = Linear velocity of x
    //v(2,0) = Linear velocity of y
    //v(3,0) = Angular velocity of q3 (Wrist pitch)
    //v(4,0) = Angular velocity of q4 (Wrist roll)
    //v(5,0) = Angular velocity of q5 (claw)

    //the negative in front of some of the joy_msg is because the direction of axis and sign of joystick is reversed

    v(1,0) = -joy_msg.axes[0]*V_MAX; //v_x
    v(2,0) = joy_msg.axes[1]*V_MAX; //v_y

    if(is_q3_const)//ability to control turntable
    {
        v(0,0) = -joy_msg.axes[3]*W_MAX; //turntable angular velocity
        v(3,0) = 0;
    }
    else //ability to control wrist pitch
    {
        v(0,0) = 0;
        v(3,0) = joy_msg.axes[4]*W_MAX; //wrist putch angular velocity
    }
    //The wrist roll and claw are zero for now and controlled somewhere else
    v(4,0) = 0;
    v(5,0) = 0;
}

void getControlScheme(const sensor_msgs::Joy& joy_msg)
{
    //The reason for such a simple function to have its own function is that the control scheme may get more complex later, and having a separate function call for the future may be beneficial
    if (joy_msg.buttons[5] == 0) //The default mode is to keep the wrist constant
        is_q3_const = true;
    else
        is_q3_const = false; //button pressed sets this to false!
}

void threshold(){
    for (int speeds = 0; speeds < dof; speeds ++)
    {
      if(abs(v(speeds,0)) < 0.002)
      {
        v(speeds,0) = 0;
      }
    }
}


void updatePosition()
{
    p_f(0,0) = v(1,0)*dt + p_i(0,0);
    p_f(1,0) = v(2,0)*dt + p_i(1,0);
    p_f(2,0) = solve_for_psi(q_i);
    //printf("x_i: %f, y_i: %f\n", p_f(0,0),p_f(1,0));
}


//Makes a float32 array that has is n_points*dof elements long.
void update_angleMsg()
{
    angleMsg.data.clear();
    for (int point = 0; point < n_points; point++)
    {
        angleMsg.data.push_back(q0_arr(point,0));
    }
    for (int point = 0; point < n_points; point++)
    {
        angleMsg.data.push_back(q1_arr(point,0));
    }
    for (int point = 0; point < n_points; point++)
    {
        angleMsg.data.push_back(q2_arr(point,0));
        //printf("WHAT?? %f", q2_arr(point,0));
        //Proof of my frustration
    }
    for (int point = 0; point < n_points; point++)
    {
        angleMsg.data.push_back(q3_arr(point,0));
    }
    for (int point = 0; point < n_points; point++)
    {
        angleMsg.data.push_back(q4_arr(point,0));
    }
    for (int point = 0; point < n_points; point++)
    {
        angleMsg.data.push_back(q5_arr(point,0));
    }
}

void MainLoop()
{
  float radius = 0;
    p_i = link_solve(q_i,a); //Find current position in space
    test_print_before();
    if (is_q3_const == 0){ //If the button is pressed, then the bottom joystick controls wrist. q3 is not constant
        for (int point = 1; point < n_points; point++)
        {
            updatePosition();//New p_f is calculated. Only x and y value.

            radius = sqrt(p_f(0,0)*p_f(0,0) + p_f(1,0)*p_f(1,0));

            if(radius > limit){ //If the position has reached its boundary, it doesnt update anymore

              q0_arr(point,0) = q_f(0,0);
              q1_arr(point,0) = q_f(1,0);
              q2_arr(point,0) = q_f(2,0);
              q3_arr(point,0) = q_f(3,0);
              q4_arr(point,0) = q_f(4,0);
              q5_arr(point,0) = q_f(5,0);

            }
            else {
              q_f(0,0) = q_i(0,0); //Turntable does not move since the bottom joystick controlling wrist now
              q_f(1,0) = solve_for_theta1(p_f, a, q_i);
              q_f(2,0) = solve_for_theta2(p_f, a, q_i);
              q_f(3,0) = solve_for_theta3(q_f, v, p_f(2,0), is_q3_const, dt);

              //*******IMPORTANT******//
              //WRIST ROLL AND CLAW ANGLES ARE HERE, BUT NO EXTRA CALCULATIONS ARE DONE AT THIS MOMEMENT. THIS IS WHY THE VALUES ARE JUST ZERO//
              q_f(4,0) = 0;
              q_f(5,0) = 0;

              //p_f(2,0) = solve_for_psi(q_f); //calculate for constantly changing phi
              //populate the angles in different step
              //Note:This will be different if cubic spline is implemeted
              q0_arr(point,0) = q_f(0,0);
              q1_arr(point,0) = q_f(1,0);
              q2_arr(point,0) = q_f(2,0);
              q3_arr(point,0) = q_f(3,0);
              q4_arr(point,0) = q_f(4,0);
              q5_arr(point,0) = q_f(5,0);

              p_i = p_f;
              q_i = q_f;
            }
        }
    }

    else{ //If the button is not pressed, wrist is automatically kept at constant and the bottom joystick controls the turntable
        for (int point = 1; point < n_points; point++)
        {
            updatePosition();//New p_f is calculated. Only x and y

            radius = sqrt(p_f(0,0)*p_f(0,0) + p_f(1,0)*p_f(1,0));
            if(radius > limit){ //If the position has reached its boundary, it doesnt update anymore

              q0_arr(point,0) = q_f(0,0);
              q1_arr(point,0) = q_f(1,0);
              q2_arr(point,0) = q_f(2,0);
              q3_arr(point,0) = q_f(3,0);
              q4_arr(point,0) = q_f(4,0);
              q5_arr(point,0) = q_f(5,0);

            }
            else{
            q_f(0,0) = solve_for_theta0(q_i(0,0), v(0,0), dt);
            q_f(1,0) = solve_for_theta1(p_f, a, q_i);
            q_f(2,0) = solve_for_theta2(p_f, a, q_i);

            q2_arr(point,0) = q_f(2,0);
            //This is out of place becasue this was the only way i was able to solve this one bug...It works fine but I have no idea why q_f(2,0) changes to its "negative" version of the angle after the solve solve_for_theta3
            //printf("BEFORE %f", q_f(2,0));
            q_f(3,0) = solve_for_theta3(q_f, v, p_f(2,0), is_q3_const, dt);
            //printf("AFTER %f", q_f(2,0));

            //*******IMPORTANT******//
            //WRIST ROLL AND CLAW ANGLES ARE HERE, BUT NO EXTRA CALCULATIONS ARE DONE AT THIS MOMEMENT. THIS IS WHY THE VALUES ARE JUST ZERO//
            q_f(4,0) = 0;
            q_f(5,0) = 0;

            //populate the angles in different step
            //Note:This will be different if cubic spline is implemeted
            q0_arr(point,0) = q_f(0,0);
            q1_arr(point,0) = q_f(1,0);

            //printf("PLEASE??? %f", q2_arr(point,0));
            q3_arr(point,0) = q_f(3,0);
            q4_arr(point,0) = q_f(4,0);
            q5_arr(point,0) = q_f(5,0);

            p_i = p_f;
            q_i = q_f;
          }
        }
    }
    test_print_after();
    update_angleMsg();
    test_print_msg();
}
