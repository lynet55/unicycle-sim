#include "simulator/test_node.hpp"


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  test_node test_node;
   
  test_node.Prepare();
  
  test_node.RunPeriodically(test_node.dt);
   
  test_node.Shutdown();
  
  return (0);
}

