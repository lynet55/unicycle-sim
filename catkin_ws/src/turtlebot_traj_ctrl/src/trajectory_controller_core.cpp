#include "controller/trajectory_controller.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  trajectory_controller trajectory_controller_node;
   
  trajectory_controller_node.Prepare();
  
  trajectory_controller_node.RunPeriodically(trajectory_controller_node.dt);
   
  trajectory_controller_node.Shutdown();
  
  return (0);
}


