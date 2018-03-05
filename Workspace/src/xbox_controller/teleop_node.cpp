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
	i = DRIVE;
	count = 0;
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
};

void StateSelect::callback(const sensor_msgs::Joy::ConstPtr& joy)
{
  ROS_INFO("Count: %d", count);
  count++;
  if (joy->buttons[6] == 1)
  	bool exit=true;

  // logic to cycle through different states
  if (joy->buttons[8]==1){
	ros::Duration(0.2).sleep();
	i++;
	if (i>=3) i=DRIVE;
  }

  	if (i==DRIVE) {
		  while (joy->buttons[6] == 1)
		  {}
		  //ROS_INFO("Button State: %d", joy->buttons[8]);		
		  ROS_INFO("Outputting to Drive");
		  
		  //pub_drive.publish("");
		  //ros::Duration(2).sleep();
	} 
	
	else if (i==ARM) {
		while (joy->buttons[6] == 1)
		{}	
		ROS_INFO("Outputting to Arm");
		teleop_twist_joy::TeleopTwistJoy joy_teleop(&nh, &nh_param);
	        //pub_arm.publish("");
	}
	
	else if (i==SCIENCE) {
		while (joy->buttons[6] == 1)
		{}	
		ROS_INFO("Outputting to Science");
		//pub_science.publish(" ");
	}
  
  else {
	//Do Nothing
	ROS_INFO("Outputting nowhere");
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "teleop_twist_joy_node");

  StateSelect state_obj;

  ros::spin();
}
