#include "geometry_msgs/Twist.h"
#include "ros/console.h"
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include <string>
#include <vector>

/*
This node implements an emergency stop. When a certain set of buttons are
pressed on the joystick
duration the autonomy mode, it sends twist messages with veolicty 0 to stop the
robot. When another
set of joystick buttons are pressed, the robot goes back to the autonomy mode
*/

class EmergencyStop {

public:
  void joy_callback(const sensor_msgs::Joy::ConstPtr &joy);
  void keep_publishing() const;
  int get_rate() const;

  EmergencyStop() {
    // get parameters
    ROS_ASSERT(ros::param::get("sub_topic", sub_topic));
    ROS_ASSERT(ros::param::get("pub_topic", pub_topic));
    ROS_ASSERT(ros::param::get("rate", rate));
    ROS_ASSERT(ros::param::get("queue_size", queue_size));
    ROS_ASSERT(ros::param::get("emergency_button_set", emergency_button_set));
    ROS_ASSERT(ros::param::get("reset_button_set", reset_button_set));

    m_sub = m_n.subscribe(sub_topic, queue_size, &EmergencyStop::joy_callback,
                          this);
    // m_pub = m_n.advertise<geometry_msgs::Twist>(pub_topic, queue_size);
    m_pub = m_n.advertise<sensor_msgs::Joy>(pub_topic, queue_size);

    emergency_mode = false;
    zero_vel_msg.linear.x = 0;
    zero_vel_msg.linear.y = 0;
    zero_vel_msg.linear.z = 0;
    zero_vel_msg.angular.x = 0;
    zero_vel_msg.angular.y = 0;
    zero_vel_msg.angular.z = 0;
  }

private:
  std::string sub_topic, pub_topic;
  int queue_size, rate;
  std::vector<int> emergency_button_set;
  std::vector<int> reset_button_set;

  ros::NodeHandle m_n;
  ros::Subscriber m_sub;
  ros::Publisher m_pub;

  bool emergency_mode;
  geometry_msgs::Twist zero_vel_msg;
};

void EmergencyStop::joy_callback(const sensor_msgs::Joy::ConstPtr &joy) {
  // ROS_WARN("received a joy");
  bool matched = true;
  if (!emergency_mode) {
    for (int i = 0; i < joy->buttons.size(); i++) {
      if (joy->buttons[i] != emergency_button_set[i]) {
        matched = false;
        // ROS_WARN("not matched");
      }
    }
  }

  if (!emergency_mode && matched) {
    // m_pub.publish(zero_vel_msg);
    emergency_mode = true;
    ROS_WARN("emergency mode enabled");
  }

  if (emergency_mode) {
    matched = true;
    for (int i = 0; i < joy->buttons.size(); i++) {
      if (joy->buttons[i] != reset_button_set[i]) {
        matched = false;
        // ROS_WARN("reset not matched");
      }
    }
    if (emergency_mode && matched) {
      emergency_mode = false;
      ROS_WARN("emergency stop disabled");
    }
  }

  // forward emergency control to teleoptwist
  if (emergency_mode) {
    m_pub.publish(joy);
  }
}

void EmergencyStop::keep_publishing() const {
  if (emergency_mode)
    m_pub.publish(zero_vel_msg);
}

int EmergencyStop::get_rate() const { return rate; }

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "e_stop");
  ROS_WARN("estop started");
  EmergencyStop e_stop;

  ros::Rate rate(e_stop.get_rate());

  while (ros::ok()) {
    // e_stop.keep_publishing();
    ros::spinOnce();
    ros::Rate(rate).sleep();
  }
}
