#ifndef TRAJECTORY_CONTROLLER_H_
#define TRAJECTORY_CONTROLLER_H_

#include "ros/ros.h"

#define RUN_PERIOD_DEFAULT 0.1
/* Used only if the actual value of the period is not retrieved from the ROS parameter server */
 
#define NAME_OF_THIS_NODE "trajectory_controller"

#include "std_msgs/Float64.h"

 
class trajectory_controller
{
  private: 
    ros::NodeHandle Handle;
    
    /* ROS topics */
    ros::Subscriber controller_subscriber;
    ros::Publisher controller_publisher;
    
    /* Parameters from ROS parameter server */
    // param_type ParamVar;

    /* ROS topic callbacks */
    void topic1_MessageCallback(const std_msgs::Float64::ConstPtr& msg);
 
    /* Node periodic task */
    void PeriodicTask(void);
    
    /* Node state variables */
    double topic1_data;
    
  public:
    double RunPeriod;
    
    void Prepare(void);
    
    void RunPeriodically(float Period);
    
    void Shutdown(void);
};

#endif /* TRAJECTORY_CONTROLLER_H_ */

