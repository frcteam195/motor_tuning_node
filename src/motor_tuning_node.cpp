#include "ros/ros.h"
#include "std_msgs/String.h"

#include <thread>
#include <string>
#include <mutex>
#include <map>

#include "rio_control_node/Motor_Configuration.h"
#include "rio_control_node/Motor_Control.h"
#include "rio_control_node/Motor_Status.h"
#include "rio_control_node/Cal_Override_Mode.h"

#include "WindowManager.hpp"

ros::NodeHandle* node;

std::map<uint32_t, rio_control_node::Motor_Config> currMotorConfigMap;
std::mutex curr_motor_config_lock;

std::map<uint32_t, rio_control_node::Motor> currMotorControlMap;
std::mutex curr_motor_control_lock;

std::map<uint32_t, rio_control_node::Motor_Info> currMotorStatusMap;
std::mutex curr_motor_status_lock;

bool motor_control_callback_has_run;
bool motor_config_callback_has_run;
std::mutex node_lock;

void motor_tuning_control_transmit()
{
	static rio_control_node::Motor_Control motorControl;
	static ros::Publisher motor_control_pub = node->advertise<rio_control_node::Motor_Control>("MotorTuningControl", 1);

	while (!motor_config_callback_has_run || !motor_control_callback_has_run)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	
	ros::Rate rate(10);

	while (ros::ok())
	{
		{
			std::scoped_lock<std::mutex> lock(curr_motor_control_lock);
			motorControl.motors.clear();
			for (auto it = currMotorControlMap.begin(); it != currMotorControlMap.end(); it++)
			{
				motorControl.motors.push_back(it->second);
			}
			motor_control_pub.publish(motorControl);
		}
		rate.sleep();
	}
}

void motor_tuning_config_transmit()
{
	static rio_control_node::Motor_Configuration motorConfiguration;
	static ros::Publisher motor_config_pub = node->advertise<rio_control_node::Motor_Configuration>("MotorTuningConfiguration", 1);

	while (!motor_config_callback_has_run || !motor_control_callback_has_run)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	ros::Rate rate(10);

	while (ros::ok())
	{
		{
			std::scoped_lock<std::mutex> lock(curr_motor_config_lock);
			motorConfiguration.motors.clear();
			for (auto it = currMotorConfigMap.begin(); it != currMotorConfigMap.end(); it++)
			{
				motorConfiguration.motors.push_back(it->second);
			}
			motor_config_pub.publish(motorConfiguration);
		}
		rate.sleep();
	}
}

void override_send_thread()
{
	static rio_control_node::Cal_Override_Mode overrideModeMsg;
	static ros::Publisher override_mode_pub = node->advertise<rio_control_node::Cal_Override_Mode>("OverrideMode", 1);

	while (!motor_config_callback_has_run || !motor_control_callback_has_run)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	WindowManager::getInstance().showWindow(currMotorConfigMap, curr_motor_config_lock, currMotorStatusMap, curr_motor_status_lock, currMotorControlMap, curr_motor_control_lock);

	ros::Rate rate(10);

	while (ros::ok())
	{
		{
			overrideModeMsg.operation_mode = rio_control_node::Cal_Override_Mode::TUNING_PIDS;
			override_mode_pub.publish(overrideModeMsg);
		}
		rate.sleep();
	}
}

void motorStatusCallback(const rio_control_node::Motor_Status &msg)
{
	std::scoped_lock<std::mutex> lock(curr_motor_status_lock);
	for (const rio_control_node::Motor_Info &currMotor : msg.motors)
	{
		currMotorStatusMap[currMotor.id] = currMotor;
	}
}

void motorCurrentConfigurationCallback(const rio_control_node::Motor_Configuration &msg)
{
	if (!motor_config_callback_has_run)
	{
		std::scoped_lock<std::mutex> lock(curr_motor_config_lock);
		for (const rio_control_node::Motor_Config &currMotor : msg.motors)
		{
			if (!currMotorConfigMap.count(currMotor.id))
			{
				currMotorConfigMap[currMotor.id] = currMotor;
			}
		}
		motor_config_callback_has_run = true;
	}
}

void motorCurrentControlCallback(const rio_control_node::Motor_Control &msg)
{
	if (!motor_control_callback_has_run)
	{
		std::scoped_lock<std::mutex> lock(curr_motor_control_lock);
		for (const rio_control_node::Motor &currMotor : msg.motors)
		{
			if (!currMotorControlMap.count(currMotor.id))
			{
				currMotorControlMap[currMotor.id] = currMotor;
			}
		}
		motor_control_callback_has_run = true;
	}
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
	std::thread motorControlSendThread(motor_tuning_control_transmit);
	std::thread motorConfigSendThread(motor_tuning_config_transmit);

	ros::Subscriber motorStatusSubscriber = node->subscribe("MotorStatus", 100, motorStatusCallback);
	ros::Subscriber motorConfigSubscriber = node->subscribe("MotorConfiguration", 100, motorCurrentConfigurationCallback);
	ros::Subscriber motorControlSubscriber = node->subscribe("MotorControl", 100, motorCurrentControlCallback);

	ros::spin();

	if (WindowManager::getInstance().getThreadHandle())
	{
		WindowManager::getInstance().getThreadHandle()->join();
	}
	overrideModeSendThread.join();
	motorConfigSendThread.join();
	motorControlSendThread.join();

	return 0;
}