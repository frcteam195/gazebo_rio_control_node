#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "rio_control_node/Motor_Control.h"
#include "rio_control_node/Motor_Status.h"

#include <thread>
#include <string>
#include <mutex>

ros::NodeHandle* node;

void gazebo_odom_callback(const nav_msgs::Odometry &msg)
{
	static ros::Publisher motor_status_pub = node->advertise<rio_control_node::Motor_Status>("MotorStatus", 1);

	rio_control_node::Motor_Status motor_status;

	for(uint32_t i = 1; i < 5; i+=3)
	{
		rio_control_node::Motor_Info motor_info;
		motor_info.id = i;
		// TBD MGT - FIXME TODO - this needs to be mathed but I'm not
		// sure what the turtlebot waffle track width is right now so I'm
		// just moving forward to get comms setup
		motor_info.sensor_velocity = msg.twist.twist.linear.x;
		motor_status.motors.push_back(motor_info);
	}
	motor_status_pub.publish(motor_status);
}

void motorControlCallback(const rio_control_node::Motor_Control &msg)
{
}

int main(int argc, char **argv)
{
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line.
	 * For programmatic remappings you can use a different version of init() which takes
	 * remappings directly, but for most command-line programs, passing argc and argv is
	 * the easiest way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "gazebo_rio_control_node");

	ros::NodeHandle n;

	node = &n;

	ros::Subscriber motorControl = node->subscribe("MotorControl", 100, motorControlCallback);
	ros::Subscriber gazeob_odom = node->subscribe("/gazebo_odom", 100, gazebo_odom_callback);

	ros::spin();
	return 0;
}