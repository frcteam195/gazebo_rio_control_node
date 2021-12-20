#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "rio_control_node/Motor_Control.h"
#include "rio_control_node/Motor_Status.h"
#include "rio_control_node/Robot_Status.h"

#include <thread>
#include <string>
#include <mutex>

ros::NodeHandle* node;

constexpr double TRACK_WIDTH = .16;

void gazebo_odom_callback(const nav_msgs::Odometry &msg)
{
	static ros::Publisher motor_status_pub = node->advertise<rio_control_node::Motor_Status>("MotorStatus", 1);
    static ros::Publisher robot_status_pub = node->advertise<rio_control_node::Robot_Status>("RobotStatus", 1);

	rio_control_node::Motor_Status motor_status;

	// double robot_velocity = (left_velocity + right_velocity) / 2.0;
	// double angular_velocity = (right_velocity - left_velocity) / TRACK_SPACING;

    double angular_velocity = msg.twist.twist.angular.z;
    double temp = angular_velocity * TRACK_WIDTH;

    double average_velocity = msg.twist.twist.linear.x;
    double left_velocity = average_velocity - (temp / 2.0);
    double right_velocity = average_velocity + (temp / 2.0);

	for(uint32_t i = 1; i < 5; i+=3)
	{
		rio_control_node::Motor_Info motor_info;
		motor_info.id = i;
		// TBD MGT - FIXME TODO - this needs to be mathed but I'm not
		// sure what the turtlebot waffle track width is right now so I'm
		// just moving forward to get comms setup
		motor_info.sensor_velocity = i == 1 ? left_velocity : right_velocity;
		motor_status.motors.push_back(motor_info);
	}
	motor_status_pub.publish(motor_status);

    rio_control_node::Robot_Status robot_status;

    robot_status.alliance = robot_status.RED;
    robot_status.robot_state = robot_status.AUTONOMOUS;
    robot_status.match_time = 0;
    robot_status.game_data = "000";

    robot_status_pub.publish(robot_status);
}

void motorControlCallback(const rio_control_node::Motor_Control &msg)
{
    static std::map<int32_t, rio_control_node::Motor> motors;
    for(std::vector<rio_control_node::Motor>::const_iterator i = msg.motors.begin();
       i != msg.motors.end();
       i++)
    {
        motors[(*i).id] = (*i);
    }

    geometry_msgs::Twist output;

    if(motors.find(1) != motors.end() && motors.find(4) != motors.end())
    {
        double left_velocity = motors[1].output_value;
        double right_velocity = motors[4].output_value;

        double robot_velocity = (left_velocity + right_velocity) / 2.0;
        double angular_velocity = (right_velocity - left_velocity) / TRACK_WIDTH;

        output.linear.x = robot_velocity;
        output.linear.y = 0;
        output.linear.z = 0;

        output.angular.x = 0;
        output.angular.y = 0;
        output.angular.z = angular_velocity;
    }
    else
    {
        output.linear.x = 0;
        output.linear.y = 0;
        output.linear.z = 0;

        output.angular.x = 0;
        output.angular.y = 0;
        output.angular.z = 0;
    }

	static ros::Publisher cmd_vel_publisher = node->advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	cmd_vel_publisher.publish(output);
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