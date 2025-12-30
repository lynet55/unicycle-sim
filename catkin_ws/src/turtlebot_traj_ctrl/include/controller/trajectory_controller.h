#ifndef TRAJECTORY_CONTROLLER_H_
#define TRAJECTORY_CONTROLLER_H_

#include "ros/ros.h"

#define RUN_PERIOD_DEFAULT 0.1
/* Used only if the actual value of the period is not retrieved from the ROS parameter server */
 
#define NAME_OF_THIS_NODE "trajectory_controller"

#include "std_msgs/Float64.h"
#include "turtlebot_simulator/TurtlebotState.h"
#include "turtlebot_simulator/ControlCommands.h"
#include "turtlebot_simulator/ReferencePoint.h"

 
class trajectory_controller
{
  private: 
    ros::NodeHandle Handle;
    
    /* ROS topics */
    ros::Subscriber controller_subscriber;
    ros::Publisher controller_publisher;
    ros::Publisher reference_publisher;
    
    /* Parameters from ROS parameter server */
    // param_type ParamVar;

    /* ROS topic callbacks */
    void controller_MessageCallback(const turtlebot_simulator::TurtlebotState::ConstPtr& msg);
 
    /* Node periodic task */
    void PeriodicTask(void);
    void control(void);

    
    /* Node state variables */
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double w = 0.0; //TODO: This one is should be removed

    double eps = 0.2;  // Epsilon distance from the trajectory
    double v_xp = 0.0;  // Velocity in x direction
    double v_yp = 0.0;  // Velocity in y direction
    double v = 0.0;  // Velocity
    double omega = 0.0;  // Angular velocity
 

    /* Controller parameters */
    double Tx = 0.1;  // Integral time constant in x direction
    double Ty = 0.1;  // Integral time constant in y direction
    double Ts = 0.01;  // Sampling time
    double Kp_x = 3.0;  // Proportional gain in x direction
    double Kp_y = 3.0;  // Proportional gain in y direction
    double v_feedforward = 1.0;  // Feedforward velocity  
    double w_feedforward = 0.0;  // Feedforward angular velocity

    double error_integral_x = 0.0;  // Integral of the error in x direction
    double error_integral_y = 0.0;  // Integral of the error in y direction
    double x_p = 0.0;  // Position in x direction
    double y_p = 0.0;  // Position in y direction
    
    /* Trajectory parameters */
    double T = 10.0;  // Period
    double a = 1.0;   // Amplitude
    
    /* Time tracking */
    double t_start = -1.0;  // Start time (initialized on first control call)
    
    /* Anti-windup limits */
    double integral_limit = 1.0;  // Max absolute value for integral terms

  public:
    double dt = RUN_PERIOD_DEFAULT;
    
    void Prepare(void);
    
    void RunPeriodically(float Period);
    
    void Shutdown(void);
    
};

#endif /* TRAJECTORY_CONTROLLER_H_ */

