#include "simulator/simulator.hpp"


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  simulator simulator;
   
  simulator.Prepare();
  
  simulator.RunPeriodically(simulator.dt);
   
  simulator.Shutdown();
  
  return (0);
}

