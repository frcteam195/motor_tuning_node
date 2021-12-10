#include "ros/ros.h"
#include "std_msgs/String.h"

#include <thread>
#include <string>
#include <mutex>

#include "rio_control_node/Motor_Configuration.h"
#include "rio_control_node/Motor_Control.h"
#include "rio_control_node/Motor_Status.h"
#include "rio_control_node/Cal_Override_Mode.h"

ros::NodeHandle* node;

void motor_tuning_control_transmit()
{
	static rio_control_node::Motor_Control motorControl;
	static ros::Publisher motor_control_pub = node->advertise<rio_control_node::Motor_Control>("MotorTuningControl", 1);

	
}

void motor_tuning_config_transmit()
{
	static rio_control_node::Motor_Configuration motorConfiguration;
	static ros::Publisher motor_config_pub = node->advertise<rio_control_node::Motor_Configuration>("MotorTuningConfiguration", 1);

	
}

void override_send_thread()
{
	static rio_control_node::Cal_Override_Mode overrideModeMsg;
	static ros::Publisher override_mode_pub = node->advertise<rio_control_node::Cal_Override_Mode>("OverrideMode", 1);
	overrideModeMsg.operation_mode = rio_control_node::Cal_Override_Mode::TUNING_PIDS;
	override_mode_pub.publish(overrideModeMsg);
}

void motorStatusCallback(const rio_control_node::Motor_Status &msg)
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
	ros::init(argc, argv, "motor_tuning_node");

	ros::NodeHandle n;

	node = &n;

	std::thread overrideModeSendThread(override_send_thread);

	ros::Subscriber motorControl = node->subscribe("MotorStatus", 100, motorStatusCallback);

	ros::spin();
	return 0;
}