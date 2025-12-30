#include "simulator/test_node.hpp"
#include "rosgraph_msgs/Clock.h"
#include <cmath>
#include <algorithm>
#include <string>


void test_node::Prepare(void)
{
	/* Node variable initialization */
	x = 0.0;
	y = 0.0;
	theta = 0.0;
	omega = 0.0;

	v_cmd = 0.0;
	omega_cmd = 0.0;
	
	/* Step input tracking initialization */
	v_step_duration = 0.0;
	omega_step_duration = 0.0;
	v_step_active = false;
	omega_step_active = false;

	/* Retrieve parameters from ROS parameter server */
	std::string FullParamName;
	
	// Get dt parameter
	FullParamName = ros::this_node::getName()+"/dt";
	if (Handle.getParam(FullParamName, dt))
	{
		ROS_INFO("Node %s: retrieved parameter %s = %.2f", 
				ros::this_node::getName().c_str(), FullParamName.c_str(), dt);
	}
	else
	{
		ROS_WARN("Node %s: unable to retrieve parameter %s, using default.", 
				ros::this_node::getName().c_str(), FullParamName.c_str());
	}
	
	/* ROS topics */
	state_subscriber = Handle.subscribe("/turtlebot/state", 1, &test_node::state_MessageCallback, this);
	commander_publisher = Handle.advertise<turtlebot_simulator::ControlCommands>("/control_commands", 1);
	
	/* ROS services */
	step_service = Handle.advertiseService("add_step", &test_node::addStepCallback, this);
	
	ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}


void test_node::RunPeriodically(float Period)
{
	ros::Rate LoopRate(1.0/Period);

	ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0/Period);

	while (ros::ok())
	{
		PeriodicTask();

		ros::spinOnce();

		LoopRate.sleep();
	}
}


void test_node::Shutdown(void)
{
	ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}


void test_node::state_MessageCallback(const turtlebot_simulator::TurtlebotState::ConstPtr& msg)
{
	/* Receive data from the topic */
	x = msg->x;
	y = msg->y;
	theta = msg->theta;
	omega = msg->w;
}

void test_node::PeriodicTask(void)
{
	/* Put here the code related to the node task */
	ros::Time now = ros::Time::now();

	/* Check if v_cmd step has expired */
	if (v_step_active && v_step_duration > 0.0) {
		double elapsed = (now - v_step_start_time).toSec();
		if (elapsed >= v_step_duration) {
			ROS_INFO("v_cmd step expired after %.3f seconds, resetting to 0", elapsed);
			v_cmd = 0.0;
			v_step_active = false;
		}
	}
	
	/* Check if omega_cmd step has expired */
	if (omega_step_active && omega_step_duration > 0.0) {
		double elapsed = (now - omega_step_start_time).toSec();
		if (elapsed >= omega_step_duration) {
			ROS_INFO("omega_cmd step expired after %.3f seconds, resetting to 0", elapsed);
			omega_cmd = 0.0;
			omega_step_active = false;
		}
	}

	/* Publish control commands */
	turtlebot_simulator::ControlCommands commander_commands;
	commander_commands.v_cmd = v_cmd;
	commander_commands.omega_cmd = omega_cmd;
	commander_publisher.publish(commander_commands);
}

bool test_node::addStepCallback(turtlebot_simulator::AddStep::Request &req,
                                         turtlebot_simulator::AddStep::Response &res)
{
	if (req.parameter == "v_cmd") {
		double old_value = v_cmd;
		v_cmd = req.step_value;  // Set to step value (not add)
		v_step_start_time = ros::Time::now();
		v_step_duration = req.duration;
		v_step_active = true;
		res.new_value = v_cmd;
		res.success = true;
		if (req.duration > 0.0) {
			res.message = "Step applied to v_cmd for " + std::to_string(req.duration) + " seconds";
			ROS_INFO("Step applied to v_cmd: %.3f -> %.3f (duration: %.3f s)", 
					 old_value, v_cmd, req.duration);
		} else {
			res.message = "Permanent step applied to v_cmd";
			ROS_INFO("Permanent step applied to v_cmd: %.3f -> %.3f", 
					 old_value, v_cmd);
		}
	}
	else if (req.parameter == "omega_cmd") {
		double old_value = omega_cmd;
		omega_cmd = req.step_value;  // Set to step value (not add)
		omega_step_start_time = ros::Time::now();
		omega_step_duration = req.duration;
		omega_step_active = true;
		res.new_value = omega_cmd;
		res.success = true;
		if (req.duration > 0.0) {
			res.message = "Step applied to omega_cmd for " + std::to_string(req.duration) + " seconds";
			ROS_INFO("Step applied to omega_cmd: %.3f -> %.3f (duration: %.3f s)", 
					 old_value, omega_cmd, req.duration);
		} else {
			res.message = "Permanent step applied to omega_cmd";
			ROS_INFO("Permanent step applied to omega_cmd: %.3f -> %.3f", 
					 old_value, omega_cmd);
		}
	}
	else {
		res.success = false;
		res.message = "Invalid parameter. Use 'v_cmd' or 'omega_cmd'";
		res.new_value = 0.0;
		ROS_WARN("Invalid step parameter requested: %s", req.parameter.c_str());
	}
	
	return true;
}



