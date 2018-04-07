#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/String.h"

/*
This code implements an algorithm that cycles through different branches when a
button is pressed.
The button used to switch between branches is the CENTER XBOX BUTTON. (The big
circular button with an X).

The order of cycle goes like : Drive --> Arm --> Science --> Drive......

*/

// A class that contains publishers and subscribers as the data members, and the
// call-back function as the member function

class StateSelect {
public:
  void callback(const sensor_msgs::Joy::ConstPtr &joy);

  StateSelect() { // Constructor
    // Subscriber
    joy_sub =
        n.subscribe<sensor_msgs::Joy>("joy", 1, &StateSelect::callback, this);

    // Publisher to four separate branches
    drive_pub = n.advertise<sensor_msgs::Joy>("drive_joy", 1);
    science_pub = n.advertise<sensor_msgs::Joy>("science_joy", 1);
    arm_pub = n.advertise<sensor_msgs::Joy>("arm_joy", 1);
    autonomy_pub = n.advertise<sensor_msgs::Joy>("autonomy_joy", 1);

    // Logic Variables Initialization
    i = DRIVE;       // Starts at the drive branch
    pressed = false; // Initially, the change button is assumed to be NOT
                     // PRESSED
    sensor_msgs::Joy empty_joy_msg; // empty joy message needed when switching states
  }

private:
  ros::NodeHandle n;

  ros::Subscriber joy_sub;
  ros::Publisher drive_pub;
  ros::Publisher science_pub;
  ros::Publisher arm_pub;
  ros::Publisher autonomy_pub;
  enum topics { DRIVE, ARM, SCIENCE, AUTONOMY };
  int i; // Variable thats holds the current state index
  bool pressed;
};

// Callback function: Called everytime there is an update on the joy-stick
void StateSelect::callback(const sensor_msgs::Joy::ConstPtr &joy) {

  // logic to cycle through different states
  if (joy->buttons[8] == 1 && !pressed) {
    // Send empty message
    if (i == DRIVE)
      drive_pub.publish(empty_joy_msg);

    else if (i == ARM)
      arm_pub.publish(empty_joy_msg);


    else if (i == SCIENCE)
      science_pub.publish(empty_joy_msg);

    else if (i == AUTONOMY)
      autonomy_pub.publish(empty_joy_msg);

    i++;
    if (i >= 4)
      i = DRIVE; // Cycle back to drive after science branch
    pressed = true;
  } else
    pressed = false;

  // The algorithm below is required to counter controller debouncing.
  if (joy->buttons[8] != 1)
    pressed = false;
  else
    pressed = true;

  if (i == DRIVE)
    drive_pub.publish(joy);

  else if (i == ARM)
    arm_pub.publish(joy);

  else if (i == SCIENCE)
    science_pub.publish(joy);

  else if (i == AUTONOMY)
    autonomy_pub.publish(joy);
}

int main(int argc, char *argv[]) {
  // Initializes Node
  ros::init(argc, argv, "teleop_node");

  StateSelect state_obj;

  ros::spin();
}
