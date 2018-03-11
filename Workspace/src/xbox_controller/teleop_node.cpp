#include "ros/ros.h"
#include "teleop_twist.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"

//typedef ::sensor_msgs::Joy_<std::allocator<void> > Joy;

class StateSelect{
  public:
  void callback(const sensor_msgs::Joy::ConstPtr& joy);
  StateSelect(){
 	joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 1, &StateSelect::callback, this);
 	drive_pub = n.advertise<sensor_msgs::Joy>("drive_joy", 1);
 	science_pub = n.advertise<sensor_msgs::Joy>("science_joy", 1);
 	arm_pub = n.advertise<sensor_msgs::Joy>("arm_joy", 1);
	i = DRIVE;
	count = 0;
	pressed=false;
  }
  private: 
	ros::NodeHandle n, nh, nh_param;

  	ros::Subscriber joy_sub;
  	ros::Publisher drive_pub;
  	ros::Publisher science_pub;
  	ros::Publisher arm_pub;
	enum topics {DRIVE,ARM,SCIENCE};
  	int i;
    int count;
    bool pressed;
};

void StateSelect::callback(const sensor_msgs::Joy::ConstPtr& joy)
{
  ROS_INFO("Count: %d", count);
  count++;
  if (joy->buttons[6] == 1)
  	bool exit=true;

  ROS_INFO("Button state: %d", pressed);

  // logic to cycle through different states
  if (joy->buttons[8]==1 && !pressed){
//	ros::Duration(0.2).sleep();
	i++; 
	if (i>=3) i=DRIVE;
	pressed=true;
  }
  else pressed=false;

  if (joy->buttons[8] != 1) pressed = false; 
  else pressed = true; 

  	if (i==DRIVE) {
		  //ROS_INFO("Button State: %d", joy->buttons[8]);		
		  ROS_INFO("Outputting to Drive");
		  
		  drive_pub.publish(joy);
		  //ros::Duration(2).sleep();
	} 
	
	else if (i==ARM) {
		ROS_INFO("Outputting to Arm");
		//teleop_twist_joy::TeleopTwistJoy joy_teleop(&nh, &nh_param);
	    	arm_pub.publish(joy);
	}
	
	else if (i==SCIENCE) {
		ROS_INFO("Outputting to Science");
		science_pub.publish(joy);
	}
  
  else {
	//Do Nothing
	ROS_INFO("Outputting nowhere");
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "teleop_node");

  StateSelect state_obj;

  ros::spin();
}
