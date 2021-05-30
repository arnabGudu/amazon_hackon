#ifndef CMD_VEL_H
#define CMD_VEL_H

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float64.h"
#include "tf/tf.h"

class platform
{
public:
	platform(ros::NodeHandle _nh);
	void callback(const sensor_msgs::Imu::ConstPtr& msg);

private:
	ros::NodeHandle nh;
	ros::Publisher pub;
	ros::Subscriber sub;
	std_msgs::Float64 command;
};

#endif
