#include <vector>
#include <iostream>
#include <math.h>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <panorama/PanoramaAction.h>

using namespace panorama;
using namespace std;

typedef actionlib::SimpleActionClient<PanoramaAction> ActionClient;

void feedbackCB(const PanoramaFeedbackConstPtr& feedback)
{
  ROS_INFO("feedbackCB(): %d frames captured", feedback->frames_captured);
}

void activeCB()
{
  ROS_INFO("activeCB(): capturing panorama");
}

void doneCB(const actionlib::SimpleClientGoalState& state,
            const PanoramaResultConstPtr& result)
{
  ROS_INFO("doneCB(): finish state is: %s, result is: %s", 
      state.toString().c_str(), result->success ? "true" : "false");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "panorama_test");
  ros::NodeHandle nh, pnh("~");

  string name;
  if (!pnh.getParam("action_server_name", name)) {
    ROS_ERROR("main(...): failed to load params");
    return -1;
  }

  ActionClient action_client(name.c_str(), true);
  ROS_INFO("Waiting for action server to start.");
  action_client.waitForServer();
  ROS_INFO("Action server started.");

  PanoramaGoal action_goal;
  int pan = 1;
  while (ros::ok()) {
    string cmd;
    getline(cin, cmd);
    if (cmd.compare("exit") == 0) {
      ROS_INFO("main(...): exiting");
      return 0;
    } else if (cmd.compare("pan") == 0) {
      action_goal.name = "pan" + to_string(pan++);
      //action_client.sendGoal(action_goal, &doneCB, &activeCB, &feedbackCB);
      action_client.sendGoal(action_goal);
      action_client.waitForResult();
    } else {
      ROS_INFO("Usage:");
      ROS_INFO("exit \tshutdown client and exit node");
      ROS_INFO("pan \tcapture panorama");
    }
  }

  ros::shutdown();
  return 0;
}
