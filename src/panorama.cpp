#include "panorama/panorama.h"

#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <ros/console.h>
#include <rosbag/bag.h>
#include <stdlib.h>
#include <math.h>

namespace rc = ros::console;
using namespace std;

template <typename T> 
int sgn(T val) 
{
  return (T(0) < val) - (val < T(0));
}

namespace panorama {

Panorama::Panorama(ros::NodeHandle nh_, ros::NodeHandle pnh_, string name) :
  listener(tf_buffer),
  as(nh_, name, false), 
  action_server_name(name)
{
  nh = nh_;
  pnh = pnh_;
  vel_pub = nh.advertise<geometry_msgs::Twist>("velocity", 10);

  // register action server methods
  as.registerGoalCallback(bind(&Panorama::goalCB, this));
  as.registerPreemptCallback(bind(&Panorama::preemptCB, this));

  // set ROS verbosity level
  bool debug = false;
  if (!pnh.getParam("debug", debug))
    ROS_WARN("[panorama] Unable to fetch debugging parameter.");
  if (debug) {
    if (rc::set_logger_level(ROSCONSOLE_DEFAULT_NAME, rc::levels::Debug)) {
      rc::notifyLoggerLevelsChanged();
      ROS_INFO("[panorama] Verbosity set to debug.");
    } else {
      ROS_ERROR("[panorama] Failed to set verbosity!");
    }
  }

  // setup approximate time synchronizer for RGBDFrames and PoseStamped msgs
  typedef openni2_xtion::RGBDFramePtr RGBDPtr;
  typedef geometry_msgs::PoseStampedConstPtr PosePtr;
  auto rgbd_cb = std::bind(&openni2_xtion::TimeFilter<RGBDPtr,PosePtr>::t1CB, 
      &time_filter, std::placeholders::_1);
  auto sync_cb = std::bind(&Panorama::syncCB, this, std::placeholders::_1, 
      std::placeholders::_2);
  pose_sub = nh.subscribe("pose", 10, 
      &openni2_xtion::TimeFilter<RGBDPtr,PosePtr>::t2CB, &time_filter); 
  xtion.registerCallback(rgbd_cb);
  time_filter.registerCallback(sync_cb);

  // fetch parameters and exit if any fails
  bool image_registration, camera_auto_settings;
  int exposure;
  if (!pnh.getParam("spin_speed", spin_speed) ||
    !pnh.getParam("number_of_frames", number_of_frames) ||
    !pnh.getParam("world_frame", world_frame) ||
    !pnh.getParam("camera_frame", camera_frame) ||
    !pnh.getParam("robot_frame", robot_frame) ||
    !pnh.getParam("save_directory", save_directory) ||
    !pnh.getParam("exposure", exposure) ||
    !pnh.getParam("camera_auto_settings", camera_auto_settings) ||
    !pnh.getParam("image_registration", image_registration)) {
    ROS_FATAL("[panorama] failed to read params from server");
    exit(EXIT_FAILURE);
  }

  // start action server
  as.start();

  // enable image registration on xtion
  if (image_registration) {
    xtion.setImageRegistration(openni::ImageRegistrationMode::
      IMAGE_REGISTRATION_DEPTH_TO_COLOR);
  }

  // update color camera settings on xtion
  if (!camera_auto_settings) {
    xtion.turnOffColorCameraAutoSettings();
    xtion.setColorCameraExposure(exposure);
  }
}

// constrain and angle to (-pi, pi]
double constrainAngle(double angle)
{
  while (angle <= -M_PI)
    angle += 2.0*M_PI;
  while (angle > M_PI)
    angle -= 2.0*M_PI;
  return angle;
}

// store pointers to color and depth images and the robot pose estimated by
// laser slam and compute the current heading of the robot
void Panorama::syncCB(const openni2_xtion::RGBDFramePtr& rgbd_msg, 
                      const geometry_msgs::PoseStampedConstPtr& pose_msg)
{
  lock_guard<mutex> lock(data_mutex);
  rgbd_ptr = rgbd_msg;
  pose_ptr = pose_msg;
  current_heading = constrainAngle(tf::getYaw(pose_msg->pose.orientation));
  /*
  ROS_DEBUG("[panorama] messages received with timestamp "
      "difference: %f", fabs(rgbd_msg->header.stamp.toSec() - 
        odom->header.stamp.toSec()));
        */
}

void Panorama::preemptCB()
{
  ROS_INFO("[panorama] %s: Preempted", action_server_name.c_str());
  as.setPreempted();
}

// prepare to execute a goal: join the main thread if it has not already 
// record the desired panorama name and spawn the panorama caputre thread
void Panorama::goalCB()
{
  if (panorama_loop_thread.joinable())
    panorama_loop_thread.join();

  // fetch goal information
  file_name = as.acceptNewGoal()->file_name;

  // start new thread of execution
  panorama_loop_thread = thread(&Panorama::captureLoop, this);
}

// compute the difference between goal_heading and the robot's current heading
// as reported by wheel odometry (wheel odometry yields the most accurate 
// heading estimate over 2pi)
double Panorama::headingDifference(double goal_heading)
{
  double diff;
  {
    lock_guard<mutex> lock(data_mutex);
    diff = goal_heading - current_heading;
  }
  return constrainAngle(diff);
}

// get the robot spinning at speed rad/s
void Panorama::sendSpinCommand(double speed)
{
  geometry_msgs::Twist vel_cmd;
  vel_cmd.angular.z = speed;
  vel_pub.publish(vel_cmd);
}

// capture a panorama and save all data required for reconstruction in a rosbag
// Information saved:
//      color, depth frames distributed about 2pi
//      the pose of the camera when each frame was captured
//      the pose of the robot when the captureLoop began
void Panorama::captureLoop()
{
  // calculate capture angles
  double start_angle;
  {
    lock_guard<mutex> lock(data_mutex);
    start_angle = current_heading;
  }
  double d_angle = sgn(spin_speed)*2.0*M_PI/((double)number_of_frames);
  vector<double> capture_angles(number_of_frames, start_angle);
  for (int i = 0; i < capture_angles.size(); ++i)
    capture_angles[i] = constrainAngle(capture_angles[i] + i*d_angle);

  // open bag
  string full_file_name = save_directory + '/' + file_name + ".bag";
  rosbag::Bag bag;
  bag.open(full_file_name.c_str(), rosbag::bagmode::Write);

  // save panorama pose
  geometry_msgs::TransformStamped t_robot_world;
  getTrans(world_frame, robot_frame, ros::Time(0), t_robot_world);
  bag.write("panorama_pose", ros::Time::now(), t_robot_world);

  // save camera info messages
  bag.write("color_camera_info", rgbd_ptr->color_info->header.stamp,
      rgbd_ptr->color_info);
  bag.write("depth_camera_info", rgbd_ptr->depth_info->header.stamp,
      rgbd_ptr->depth_info);

  // capture panorama frames
  ros::Rate loop_rate(30);
  int frame = 1;
  while (frame <= number_of_frames) {

    // check if action has been cancelled
    if (!as.isActive() || as.isPreemptRequested()) {
      sendSpinCommand(0.0);
      bag.close();
      return;
    }

    // publish velocity command
    sendSpinCommand(spin_speed);

    // save data if the robot has reached the desired frame heading
    if (sgn(headingDifference(capture_angles[frame])) != sgn(spin_speed)) {
      
      // fetch synchornized sensor data
      ros::Time frame_stamp;
      {
        lock_guard<mutex> lock(data_mutex);
        // save image data to bag
        bag.write("color", rgbd_ptr->color->header.stamp, rgbd_ptr->color);
        bag.write("depth", rgbd_ptr->depth->header.stamp, rgbd_ptr->depth);
        frame_stamp = pose_ptr->header.stamp;
      }

      // save camera pose
      geometry_msgs::TransformStamped slam_camera_pose;
      getTrans(world_frame, camera_frame, frame_stamp, slam_camera_pose);
      bag.write("camera_pose", frame_stamp, slam_camera_pose);

      panorama::PanoramaFeedback feedback;
      feedback.frames_captured = frame++;
      as.publishFeedback(feedback);
    }

    loop_rate.sleep();
  }
  bag.close();
  sendSpinCommand(0.0);

  panorama::PanoramaResult result;
  result.full_file_name = full_file_name;
  as.setSucceeded(result);
}

} // panorama namespace
