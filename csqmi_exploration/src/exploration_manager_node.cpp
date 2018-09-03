#include "csqmi_exploration/exploration_manager.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "csqmi_exploration_manager");
  ros::NodeHandle nh, pnh("~");

  csqmi_exploration::ExplorationManager manager(pnh, nh);
  manager.explorationLoop();

  return 0;
}
