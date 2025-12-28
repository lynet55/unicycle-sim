#ifndef TRAJECTORY_CONTROLLER_H_
#define TRAJECTORY_CONTROLLER_H_

#include "ros/ros.h"

#define RUN_PERIOD_DEFAULT 0.1
/* Used only if the actual value of the period is not retrieved from the ROS parameter server */
 
#define NAME_OF_THIS_NODE "trajectory_controller"

#include "std_msgs/Float64.h"
#include "turtlebot_simulator/TurtlebotState.h"
#include "turtlebot_simulator/ControlCommands.h"
#include "rosgraph_msgs/Clock.h"
#include "trajectory_generator.hpp"

 
class trajectory_controller
{
  private: 
    ros::NodeHandle Handle;
    
    /* ROS topics */
    ros::Subscriber controller_subscriber;
    ros::Publisher controller_publisher;
    ros::Publisher reference_publisher;
    ros::Publisher clock_publisher;
    
    /* Parameters from ROS parameter server */
    // param_type ParamVar;

    /* ROS topic callbacks */
    void controller_MessageCallback(const turtlebot_simulator::TurtlebotState::ConstPtr& msg);
 
    /* Node periodic task */
    void PeriodicTask(void);
    void control(void);
    
    /* Node state variables */
    double x;
    double y;
    double w;
    double t_prev;  // Previous timestamp for integral control
    double v_feedforward;  // Feedforward velocity

    /*Controller params */
    double Kp_x = 5.0;
	  double Kp_y = 5.0;
	  double Ki_x = 0.1;
	  double Ki_y = 0.1;
    
    /* Trajectory parameters */
    double T;  // Period
    double a;  // Amplitude
    Trajectory trajectory;  // Trajectory object
    
  public:
    double RunPeriod;
    
    void Prepare(void);
    
    void RunPeriodically(float Period);
    
    void Shutdown(void);
    
};

#endif /* TRAJECTORY_CONTROLLER_H_ */

