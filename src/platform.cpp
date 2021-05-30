#include <platform.h>
#include <iostream>

using namespace std;

platform::platform(ros::NodeHandle _nh) : nh(_nh) {
	pub = nh.advertise<std_msgs::Float64>("/stair_climber/joint_position_controller/command", 1000);
	sub = nh.subscribe("/imu", 1000, &platform::callback, this);
}

void platform::callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  command.data = roll-1.57;
  pub.publish(command);
	
  cout<<roll<<" "<<pitch<<" "<<yaw<<endl;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "platform");
	ros::NodeHandle nh;

	platform p(nh);

	ros::spin();
	return 0;
}
