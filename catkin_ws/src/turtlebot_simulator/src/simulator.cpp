#include "simulator/simulator.hpp"
#include "simulator/simulator_odefun.hpp"
#include <vector>
#include <boost/numeric/odeint.hpp>


using namespace std;
using namespace boost::numeric::odeint;

void simulator::Prepare(void)
{
	RunPeriod = RUN_PERIOD_DEFAULT;
	dt_ = 0.01;  // Default timestep
	v_cmd = 0.0;
	omega_cmd = 0.0;

	/* Retrieve parameters from ROS parameter server */
	if (Handle.getParam(ros::this_node::getName()+"/run_period", RunPeriod))
	{
		ROS_INFO("Node %s: retrieved parameter run_period = %.2f", 
				ros::this_node::getName().c_str(), RunPeriod);
	}
	else
	{
		ROS_WARN("Node %s: unable to retrieve parameter run_period, using default = %.2f", 
				ros::this_node::getName().c_str(), RunPeriod);
	}

	if (Handle.getParam(ros::this_node::getName()+"/T_a", T_a))
	{
		ROS_INFO("Node %s: retrieved parameter T_a = %.2f", 
				ros::this_node::getName().c_str(), T_a);
	}
	else
	{
		ROS_WARN("Node %s: unable to retrieve parameter T_a, using default = %.2f", 
				ros::this_node::getName().c_str(), T_a);
	}

	if (Handle.getParam(ros::this_node::getName()+"/a", a))
	{
		ROS_INFO("Node %s: retrieved parameter a = %.2f", 
				ros::this_node::getName().c_str(), a);
	}
	else
	{
		ROS_WARN("Node %s: unable to retrieve parameter a, using default = %.2f", 
				ros::this_node::getName().c_str(), a);
	}

	if (Handle.getParam(ros::this_node::getName()+"/dt", dt_))
	{
		ROS_INFO("Node %s: retrieved parameter dt = %.2f", 
				ros::this_node::getName().c_str(), dt_);
	}
	else
	{
		ROS_WARN("Node %s: unable to retrieve parameter dt, using default = %.2f", 
				ros::this_node::getName().c_str(), dt_);
	}


	/* ROS topics */
	simulator_subscriber = Handle.subscribe("/control_commands", 1, &simulator::simulator_MessageCallback, this);
	simulator_publisher = Handle.advertise<turtlebot_simulator::TurtlebotState>("/turtlebot/state", 1);
	clock_publisher = Handle.advertise<rosgraph_msgs::Clock>("/clock", 1);

	/* Node variable initialization */
	simulator_ptr = new UnicycleRobot(dt_);	
	simulator_ptr->setInitalState(0.0, 0.0, 0.0, 0.0, 0.0);
	simulator_ptr->setModelParams(a, T_a);

	ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}


void simulator::RunPeriodically(float Period)
{
	ros::Rate LoopRate(1.0/Period);
	ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0/Period);

	sleep(1.0);

	while (ros::ok())
	{
		PeriodicTask();
		ros::spinOnce();
		LoopRate.sleep();
	}
}


void simulator::Shutdown(void)
{
	ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void simulator::simulator_MessageCallback(const turtlebot_simulator::ControlCommands::ConstPtr& msg)
{
	/* Receive data from the topic */
	v_cmd = msg->v_cmd;
	omega_cmd = msg->omega_cmd;
}

void simulator::PeriodicTask(void)
{
	/* Put here the code related to the node task */
	simulator_ptr->setInputValues(v_cmd, omega_cmd);
	simulator_ptr->integrate();

	/* Print simulator time every 5 sec*/
	double time;
	simulator_ptr->getTime(time);
	ROS_INFO("Simulator time: %.2f seconds, v_cmd: %.2f, omega_cmd: %.2f", time, v_cmd, omega_cmd);

	/* Publish turtlebot state */
	double x, y, theta, v, omega;
	simulator_ptr->getState(x, y, theta, v, omega);
	
	turtlebot_simulator::TurtlebotState outputMsg;
	outputMsg.x = x;
	outputMsg.y = y;
	outputMsg.w = omega;
	simulator_publisher.publish(outputMsg);

	/* Publish clock */
	rosgraph_msgs::Clock clockMsg;
	clockMsg.clock = ros::Time(time);
	clock_publisher.publish(clockMsg);
}

