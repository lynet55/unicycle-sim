#include "controller/trajectory_controller.h"
#include "rosgraph_msgs/Clock.h"
#include <cmath>
#include <algorithm>


void trajectory_controller::Prepare(void)
{
	/* Node variable initialization */
	RunPeriod = RUN_PERIOD_DEFAULT;
	x = 0.0;
	y = 0.0;
	theta = 0.0;
	w = 0.0;
	v_feedforward = 1.0;
	t_prev = ros::Time::now().toSec();
	T = 10.0;  // Default period
	a = 1.0;   // Default amplitude
	
	// Initialize control variables to avoid NaN
	v = 0.0;
	omega = 0.0;
	v_xp = 0.0;
	v_yp = 0.0;
	x_p = 0.0;
	y_p = 0.0;
	eps = 0.2;  // Default epsilon value

	/* Retrieve parameters from ROS parameter server */
	std::string FullParamName;
	
	// Get run_period parameter
	FullParamName = ros::this_node::getName()+"/run_period";
	if (Handle.getParam(FullParamName, RunPeriod))
	{
		ROS_INFO("Node %s: retrieved parameter %s = %.2f", 
				ros::this_node::getName().c_str(), FullParamName.c_str(), RunPeriod);
	}
	else
	{
		ROS_WARN("Node %s: unable to retrieve parameter %s, using default.", 
				ros::this_node::getName().c_str(), FullParamName.c_str());
	}
	
	// Get trajectory period T
	FullParamName = ros::this_node::getName()+"/T";
	if (Handle.getParam(FullParamName, T))
	{
		ROS_INFO("Node %s: retrieved parameter T = %.2f", 
				ros::this_node::getName().c_str(), T);
	}
	else
	{
		ROS_WARN("Node %s: unable to retrieve parameter T, using default = %.2f", 
				ros::this_node::getName().c_str(), T);
	}
	
	// Get trajectory amplitude a
	FullParamName = ros::this_node::getName()+"/a";
	if (Handle.getParam(FullParamName, a))
	{
		ROS_INFO("Node %s: retrieved parameter a = %.2f", 
				ros::this_node::getName().c_str(), a);
	}
	
	else
	{
		ROS_WARN("Node %s: unable to retrieve parameter a, using default = %.2f", 
				ros::this_node::getName().c_str(), a);
	}
	
	// Generate trajectory after parameters are loaded
	trajectory = Trajectory(T, a);
	trajectory.generateTrajectory(100, a);
	
	// Verify trajectory was generated successfully
	ROS_INFO("Node %s: Generated trajectory with %zu points (T=%.2f, a=%.2f)", 
			ros::this_node::getName().c_str(), trajectory.size(), T, a);
	if (trajectory.size() > 0) {
		std::pair<double, double> firstPoint = trajectory[0];
		ROS_INFO("  First point: (%.2f, %.2f)", firstPoint.first, firstPoint.second);
	}

	/* ROS topics */
	controller_subscriber = Handle.subscribe("/turtlebot/state", 1, &trajectory_controller::controller_MessageCallback, this);
	controller_publisher = Handle.advertise<turtlebot_simulator::ControlCommands>("/control_commands", 1);
	reference_publisher = Handle.advertise<turtlebot_simulator::ReferencePoint>("/turtlebot/reference", 1);
	ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
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
	
	// Get trajectory reference point based on time
	// Map time to trajectory index: t modulo T, then scale to number of points
	size_t numPoints = trajectory.size();
	ROS_INFO("Controller: t=%.2f, numPoints=%zu, x=%.2f, y=%.2f", t, numPoints, x, y);
	if (numPoints > 0)
	{

		// double t_normalized = fmod(t, T);  // Get time within one period
		// size_t index = static_cast<size_t>((t_normalized / T) * numPoints);
		// index = std::min(index, numPoints - 1);  // Clamp to valid range
		

		// // Get current trajectory reference position
		// double x_d = trajectory[index].first;
		// double y_d = trajectory[index].second;

				
		// Compute trajectory velocity (feedforward term) using numerical derivative
		// size_t next_index = (index + 1) % numPoints;  // Wrap around for periodic trajectory
		// double x_d_next = trajectory[next_index].first;
		// double y_d_next = trajectory[next_index].second;
		// double dt = T / numPoints;  // Time step between trajectory points

	double pi = M_PI;
	double xp_d = a * sin(2 * pi * t / T);
	double yp_d = a * sin(2 * pi * t / T) * cos(2 * pi * t / T);
	double xpdot_d = a * 2 * pi / T * cos(2 * pi * t / T);
	double ypdot_d = a * (2 * pi / T) * cos(4 * pi * t / T);

	// Control point ahead of robot
	x_p = x + eps * cos(theta);
	y_p = y + eps * sin(theta);

	// Feedback + Feedforward control law
	v_xp = xpdot_d + Kp_x * (xp_d - x_p);  // Feedforward + feedback
	v_yp = ypdot_d + Kp_y * (yp_d - y_p);  // Feedforward + feedback

	v = v_xp * cos(theta) + v_yp * sin(theta);
	omega = (v_yp * cos(theta) - v_xp * sin(theta)) / eps;

	ROS_INFO("Controller time: %.2f seconds, velocity_command: %.2f, angular_velocity_command: %.2f", t, v, omega);


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
}


