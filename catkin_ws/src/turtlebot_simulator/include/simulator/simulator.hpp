#ifndef simulator_H_
#define simulator_H_

#include "ros/ros.h"
#include "simulator/simulator_odefun.hpp"
#include "turtlebot_simulator/ControlCommands.h"
#include "turtlebot_simulator/TurtlebotState.h"
#include "rosgraph_msgs/Clock.h"

#define RUN_PERIOD_DEFAULT 0.1
/* Used only if the actual value of the period is not retrieved from the ROS parameter server */
 
#define NAME_OF_THIS_NODE "simulator"
 
class simulator
{
  private: 
    ros::NodeHandle Handle;
    
    /* ROS topics */
    ros::Subscriber simulator_subscriber;
    ros::Publisher simulator_publisher;
    ros::Publisher clock_publisher;
    
    /* Parameters from ROS parameter server */
    double a;
    double T_a;

    /* ROS topic callbacks */
    void simulator_MessageCallback(const turtlebot_simulator::ControlCommands::ConstPtr& msg);
 
    /* Node periodic task */
    void PeriodicTask(void);
    
    /* Node state variables */
    UnicycleRobot* simulator_ptr;
    double v_cmd;
    double omega_cmd;

  public:
    double RunPeriod = RUN_PERIOD_DEFAULT;
    double dt = 0.01;
    
    void Prepare(void);
    
    void RunPeriodically(float Period);
    
    void Shutdown(void);
};

#endif /* simulator_H_ */
