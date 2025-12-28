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
	w = 0.0;
	v_feedforward = 1.0;
	t_prev = ros::Time::now().toSec();
	T = 10.0;  // Default period
	a = 1.0;   // Default amplitude

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

	/* ROS topics */
	controller_subscriber = Handle.subscribe("/turtlebot/state", 1, &trajectory_controller::controller_MessageCallback, this);
	controller_publisher = Handle.advertise<turtlebot_simulator::ControlCommands>("/control_commands", 1);
	reference_publisher = Handle.advertise<turtlebot_simulator::TurtlebotState>("/turtlebot/reference", 1);
	clock_publisher = Handle.advertise<rosgraph_msgs::Clock>("/clock", 1);

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

		double t_normalized = fmod(t, T);  // Get time within one period
		size_t index = static_cast<size_t>((t_normalized / T) * numPoints);
		index = std::min(index, numPoints - 1);  // Clamp to valid range
		
		double x_ref = trajectory[index].first;
		double y_ref = trajectory[index].second;

		double dt = t - t_prev;
		double u_vx = v_feedforward + Kp_x * (x_ref - x) + Ki_x * (x_ref - x) * dt;
		double u_vy = Kp_y * (y_ref - y) + Ki_y * (y_ref - y) * dt;
		double u_w_z = 0.0; /*v sin⁻1(y / abs_v ) theta dot */
		
	/* Publish reference trajectory */
	turtlebot_simulator::TurtlebotState referenceMsg;
	referenceMsg.x = x_ref;
	referenceMsg.y = y_ref;
	referenceMsg.w = 0.0;  // Not used for reference
	reference_publisher.publish(referenceMsg);
		
	/* Publish control commands */
	turtlebot_simulator::ControlCommands control_commands;
	control_commands.v_cmd = u_vx;
	control_commands.omega_cmd = u_w_z;
	controller_publisher.publish(control_commands);
		
		// Publish clock 
		rosgraph_msgs::Clock clockMsg;
		clockMsg.clock = ros::Time(t);
		clock_publisher.publish(clockMsg);
	}
}


