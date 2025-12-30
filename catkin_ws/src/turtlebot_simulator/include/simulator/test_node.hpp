#ifndef TEST_NODE_HPP_
#define TEST_NODE_HPP_

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "turtlebot_simulator/TurtlebotState.h"
#include "turtlebot_simulator/ControlCommands.h"
#include "turtlebot_simulator/ReferencePoint.h"
#include "turtlebot_simulator/AddStep.h"

#define RUN_PERIOD_DEFAULT 0.1
#define NAME_OF_THIS_NODE "test_node"

class test_node
{
  private: 
    ros::NodeHandle Handle;

    /* ROS topics */
    ros::Subscriber state_subscriber;
    ros::Publisher commander_publisher;

    /* ROS services */
    ros::ServiceServer step_service;

    /* ROS topic callbacks */
    void state_MessageCallback(const turtlebot_simulator::TurtlebotState::ConstPtr& msg);

    /* ROS service callbacks */
    bool addStepCallback(turtlebot_simulator::AddStep::Request &req,
                        turtlebot_simulator::AddStep::Response &res);
 
    /* Node periodic task */
    void PeriodicTask(void);
    
    /* Node state variables */
    double x;
    double y;
    double theta;  // Heading angle
    double omega;  // Angular velocity

    /*Controller params */
    double v_cmd;  // Velocity
    double omega_cmd;  // Angular velocity
    
    /* Step input tracking */
    ros::Time v_step_start_time;
    ros::Time omega_step_start_time;
    double v_step_duration;      // Duration in seconds (0 = permanent)
    double omega_step_duration;
    bool v_step_active;
    bool omega_step_active;
    
  public:
    double dt = RUN_PERIOD_DEFAULT;
    
    void Prepare(void);
    
    void RunPeriodically(float Period);
    
    void Shutdown(void);
    
};

#endif // TEST_NODE_HPP_

