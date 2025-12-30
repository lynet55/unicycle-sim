#include "controller/trajectory_controller.h"
#include "rosgraph_msgs/Clock.h"
#include <cmath>
#include <algorithm>


void trajectory_controller::Prepare(void)
{
	/* Retrieve parameters from ROS parameter server */
	std::string FullParamName;
	std::string nodeName = ros::this_node::getName();
	
	// Get run_period parameter (dt)
	FullParamName = nodeName + "/run_period";
	if (Handle.getParam(FullParamName, dt))
		ROS_INFO("Node %s: retrieved parameter run_period = %.3f", nodeName.c_str(), dt);
	else
		ROS_WARN("Node %s: unable to retrieve parameter run_period, using default = %.3f", nodeName.c_str(), dt);
	
	// Get trajectory period T
	FullParamName = nodeName + "/T";
	if (Handle.getParam(FullParamName, T))
		ROS_INFO("Node %s: retrieved parameter T = %.2f", nodeName.c_str(), T);
	else
		ROS_WARN("Node %s: unable to retrieve parameter T, using default = %.2f", nodeName.c_str(), T);
	
	// Get trajectory amplitude a
	FullParamName = nodeName + "/a";
	if (Handle.getParam(FullParamName, a))
		ROS_INFO("Node %s: retrieved parameter a = %.2f", nodeName.c_str(), a);
	else
		ROS_WARN("Node %s: unable to retrieve parameter a, using default = %.2f", nodeName.c_str(), a);
	
	// Get proportional gain Kp_x
	FullParamName = nodeName + "/Kp_x";
	if (Handle.getParam(FullParamName, Kp_x))
		ROS_INFO("Node %s: retrieved parameter Kp_x = %.2f", nodeName.c_str(), Kp_x);
	else
		ROS_WARN("Node %s: unable to retrieve parameter Kp_x, using default = %.2f", nodeName.c_str(), Kp_x);
	
	// Get proportional gain Kp_y
	FullParamName = nodeName + "/Kp_y";
	if (Handle.getParam(FullParamName, Kp_y))
		ROS_INFO("Node %s: retrieved parameter Kp_y = %.2f", nodeName.c_str(), Kp_y);
	else
		ROS_WARN("Node %s: unable to retrieve parameter Kp_y, using default = %.2f", nodeName.c_str(), Kp_y);
	
	// Get integral time constant Tx
	FullParamName = nodeName + "/Tx";
	if (Handle.getParam(FullParamName, Tx))
		ROS_INFO("Node %s: retrieved parameter Tx = %.2f", nodeName.c_str(), Tx);
	else
		ROS_WARN("Node %s: unable to retrieve parameter Tx, using default = %.2f", nodeName.c_str(), Tx);
	
	// Get integral time constant Ty
	FullParamName = nodeName + "/Ty";
	if (Handle.getParam(FullParamName, Ty))
		ROS_INFO("Node %s: retrieved parameter Ty = %.2f", nodeName.c_str(), Ty);
	else
		ROS_WARN("Node %s: unable to retrieve parameter Ty, using default = %.2f", nodeName.c_str(), Ty);
	
	// Get sampling time Ts
	FullParamName = nodeName + "/Ts";
	if (Handle.getParam(FullParamName, Ts))
		ROS_INFO("Node %s: retrieved parameter Ts = %.3f", nodeName.c_str(), Ts);
	else
		ROS_WARN("Node %s: unable to retrieve parameter Ts, using default = %.3f", nodeName.c_str(), Ts);
	
	// Get epsilon distance eps
	FullParamName = nodeName + "/eps";
	if (Handle.getParam(FullParamName, eps))
		ROS_INFO("Node %s: retrieved parameter eps = %.2f", nodeName.c_str(), eps);
	else
		ROS_WARN("Node %s: unable to retrieve parameter eps, using default = %.2f", nodeName.c_str(), eps);
	
	// Get feedforward velocity
	FullParamName = nodeName + "/v_feedforward";
	if (Handle.getParam(FullParamName, v_feedforward))
		ROS_INFO("Node %s: retrieved parameter v_feedforward = %.2f", nodeName.c_str(), v_feedforward);
	else
		ROS_WARN("Node %s: unable to retrieve parameter v_feedforward, using default = %.2f", nodeName.c_str(), v_feedforward);
	
	// Get feedforward angular velocity
	FullParamName = nodeName + "/w_feedforward";
	if (Handle.getParam(FullParamName, w_feedforward))
		ROS_INFO("Node %s: retrieved parameter w_feedforward = %.2f", nodeName.c_str(), w_feedforward);
	else
		ROS_WARN("Node %s: unable to retrieve parameter w_feedforward, using default = %.2f", nodeName.c_str(), w_feedforward);

	/* ROS topics */
	controller_subscriber = Handle.subscribe("/turtlebot/state", 1, &trajectory_controller::controller_MessageCallback, this);
	controller_publisher = Handle.advertise<turtlebot_simulator::ControlCommands>("/control_commands", 1);
	reference_publisher = Handle.advertise<turtlebot_simulator::ReferencePoint>("/turtlebot/reference", 1);
	ROS_INFO("Node %s ready to run.", nodeName.c_str());
}


void trajectory_controller::RunPeriodically(float Period)
{
	ros::Rate LoopRate(1.0/Period);

	ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0/Period);

	while (ros::ok())
	{
		control();

		ros::spinOnce();

		LoopRate.sleep();
	}
}


void trajectory_controller::Shutdown(void)
{
	ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());

}


void trajectory_controller::controller_MessageCallback(const turtlebot_simulator::TurtlebotState::ConstPtr& msg)
{
	/* Receive data from the topic */
	x = msg->x;
	y = msg->y;
	theta = msg->theta;
	w = msg->w;
}

void trajectory_controller::control(void)
{
	/* Put here the code related to the node task */
	double t = ros::Time::now().toSec();


	double pi = M_PI;
	double xp_d = a * sin(2 * pi * t / T);
	double yp_d = a * sin(2 * pi * t / T) * cos(2 * pi * t / T);
	double xpdot_d = a * 2 * pi / T * cos(2 * pi * t / T);
	double ypdot_d = a * (2 * pi / T) * cos(4 * pi * t / T);

	// Control point ahead of robot
	x_p = x + eps * cos(theta);
	y_p = y + eps * sin(theta);

	// step commmand
	xp_d = 1.0;
	yp_d = 1.0;

	double ex = xp_d - x_p;
	double ey = yp_d - y_p;

	// Feedback + Feedforward control law
	v_xp = xpdot_d + Kp_x * (ex + (1/Tx) * error_integral_x);
	v_yp = ypdot_d + Kp_y * (ey + (1/Ty) * error_integral_y);
	
	v = v_xp * cos(theta) + v_yp * sin(theta);
	omega = (v_yp * cos(theta) - v_xp * sin(theta)) / eps;

	/* Publish reference trajectory */
	turtlebot_simulator::ReferencePoint referenceMsg;
	referenceMsg.x = xp_d;
	referenceMsg.y = yp_d;
	reference_publisher.publish(referenceMsg);
		
	/* Publish control commands */
	turtlebot_simulator::ControlCommands control_commands;
	control_commands.v_cmd = v;
	control_commands.omega_cmd = omega;
	controller_publisher.publish(control_commands);

}


