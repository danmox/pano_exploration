#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>

#include <actionlib/client/simple_action_client.h>
#include <panorama/PanoramaAction.h>

using namespace panorama;
using namespace std;

typedef actionlib::SimpleActionClient<PanoramaAction> ActionClient;

int kfd;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

/*
void overwriteLineWith(const string msg)
{
  cout << "\33[2K\r";
  cout << msg << flush;
}
*/

void feedbackCB(const PanoramaFeedbackConstPtr& feedback)
{
  printf("Captured frame %d of 72\n", feedback->frames_captured);
}

void activeCB()
{
  printf("\nCapturing panorama...\n");
}

void doneCB(const actionlib::SimpleClientGoalState& state,
            const PanoramaResultConstPtr& result)
{
  printf("\nPanorama completed with %s", state.toString().c_str());
  printf(" and saved to %s\n\n", result->full_file_name.c_str());
}

int pan = 1;

int main(int argc, char** argv)
{
  if (argc < 2) {
    printf("Usage: rosrun panorama cli <robot#>\n");
    exit(EXIT_FAILURE);
  }

  ros::init(argc, argv, "panorama_action_cli");

  string server_name = "/robot" + string(argv[1]) + "/panorama";
  ActionClient ac(server_name.c_str(), true);
  printf("Waiting for action server to start: %s\n", server_name.c_str());
  ac.waitForServer();
  printf("%s is ready\n", server_name.c_str());

  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  char c;
  while (ros::ok()) {
    if (read(kfd, &c, 1) < 0) {
      ROS_FATAL("Error reading from keyboard");
      exit(EXIT_FAILURE);
    }

    switch(c) {
      case 'p': {
          PanoramaGoal ag;
          ag.file_name = "robot" + string(argv[1]) + "pan" + to_string(pan++);
          ac.sendGoal(ag, &doneCB, &activeCB, &feedbackCB);
          ac.waitForResult();
        }
      //case 'q' : return 0;
      default : break;
    }
  }

  return 0;
}
