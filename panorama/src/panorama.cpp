#include "panorama/panorama.h"

#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <ros/console.h>
#include <rosbag/bag.h>
#include <stdlib.h>
#include <math.h>

namespace rc = ros::console;
using namespace std;
using namespace geometry_msgs;

template <typename T>
int sgn(T val)
{
  return (T(0) < val) - (val < T(0));
}

PoseStamped transToPose(const TransformStamped& trans)
{
  PoseStamped pose;
  pose.header = trans.header;
  pose.pose.orientation = trans.transform.rotation;
  pose.pose.position.x = trans.transform.translation.x;
  pose.pose.position.y = trans.transform.translation.y;
  pose.pose.position.z = trans.transform.translation.z;
  return pose;
}

namespace panorama {

Panorama::Panorama(ros::NodeHandle nh_, ros::NodeHandle pnh_, string name) :
  listener(tf_buffer),
  as(nh_, name, false),
  action_server_name(name)
{
  nh = nh_;
  pnh = pnh_;
  vel_pub = nh.advertise<Twist>("velocity", 10);

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
  auto rgbd_cb = bind(&openni2_xtion::TimeFilter<RGBDPtr,PosePtr>::t1CB, &time_filter, _1);
  auto sync_cb = bind(&Panorama::syncCB, this, _1, _2);
  pose_sub = nh.subscribe("pose", 10, &openni2_xtion::TimeFilter<RGBDPtr,PosePtr>::t2CB, &time_filter);
  xtion.registerCallback(rgbd_cb);
  time_filter.registerCallback(sync_cb);

  // fetch parameters and exit if any fails
  bool image_registration, camera_auto_settings;
  int exposure;
  if (!pnh.getParam("spin_speed", spin_speed) ||
      !pnh.getParam("number_of_frames", number_of_frames) ||
      !pnh.getParam("continuous_capture", continuous_capture) ||
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

  // manually set color camera exposure
  if (!camera_auto_settings) {
    xtion.enableColorCameraAutoSettings(camera_auto_settings);
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
                      const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
  lock_guard<mutex> lock(data_mutex);
  rgbd_ptr = rgbd_msg;
  pose_timestamp = pose_msg->header.stamp;
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
double Panorama::headDiff(double goal_heading)
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
  Twist vel_cmd;
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
  vector<double> capture_angles(number_of_frames, start_angle);
  if (!continuous_capture) {
    double d_angle = sgn(spin_speed)*2.0*M_PI/((double)number_of_frames);
    for (int i = 0; i < capture_angles.size(); ++i)
      capture_angles[i] = constrainAngle(capture_angles[i] + i*d_angle);
  }

  // open bag
  ROS_DEBUG("[panorama] openning bag");
  string full_file_name = save_directory + '/' + file_name + ".bag";
  rosbag::Bag bag;
  bag.open(full_file_name.c_str(), rosbag::bagmode::Write);

  // save panorama pose
  ROS_DEBUG("[panorama] saving panorama pose");
  TransformStamped t_robot_world;
  if(!getTrans(world_frame, robot_frame, ros::Time(0), t_robot_world)) {
    ROS_ERROR("[panorama] unable to save panorama pose: aborting");
    bag.close();
    sendSpinCommand(0.0);
    panorama::PanoramaResult result;
    as.setAborted(result);
    return;
  }
  PoseStamped pano_pose = transToPose(t_robot_world);
  bag.write("/panorama_pose", pano_pose.header.stamp, pano_pose);

  // save camera info messages
  ROS_DEBUG("[panorama] saving camera info messages");
  bag.write("/color_camera_info", rgbd_ptr->color_info->header.stamp,
      rgbd_ptr->color_info);
  bag.write("/depth_camera_info", rgbd_ptr->depth_info->header.stamp,
      rgbd_ptr->depth_info);

  // capture panorama frames
  ros::Rate loop_rate(30);
  int frame = 1;
  double last_frame_heading(current_heading), total_turn(0.0);
  bool complete = false;
  ROS_DEBUG("[panorama] beginning frame loop");
  while (!complete && nh.ok()) {
    ROS_DEBUG("[panorama] in capture loop");

    // check if action has been cancelled
    if (!as.isActive() || as.isPreemptRequested()) {
      ROS_DEBUG("[panorama] goal cancelled, exiting capture loop");
      sendSpinCommand(0.0);
      bag.close();
      return;
    }

    ROS_DEBUG("[panorama] sending spin command");
    sendSpinCommand(spin_speed);

    // determine if a frame should be captured
    bool capture_frame = false;
    if (!continuous_capture) {
      capture_frame = sgn(headDiff(capture_angles[frame])) != sgn(spin_speed);
    } else {
      // in continuous mode, only capture frames when robot is turning
      double tmp = -sgn(spin_speed)*headDiff(last_frame_heading);
      capture_frame = -sgn(spin_speed)*headDiff(last_frame_heading) > 0.01;
    }

    // save data if the robot has reached the desired frame heading
    if (capture_frame) {

      // fetch pose timestamp
      ros::Time frame_stamp;
      {
        lock_guard<mutex> lock(data_mutex);
        frame_stamp = pose_timestamp;
      }

      // save camera pose
      TransformStamped camera_trans;
      if(!getTrans(world_frame, camera_frame, frame_stamp, camera_trans)) {
        loop_rate.sleep();
        continue;
      }
      PoseStamped camera_pose = transToPose(camera_trans);
      bag.write("/camera_pose", camera_pose.header.stamp, camera_pose);

      // save images
      {
        lock_guard<mutex> lock(data_mutex);
        // save image data to bag
        bag.write("/color", rgbd_ptr->color->header.stamp, rgbd_ptr->color);
        bag.write("/depth", rgbd_ptr->depth->header.stamp, rgbd_ptr->depth);
      }

      // update the total angular displacement since robot started pano
      double this_frame_heading;
      this_frame_heading = constrainAngle(tf::getYaw(camera_trans.transform.rotation));
      total_turn += constrainAngle(this_frame_heading - last_frame_heading);
      last_frame_heading = this_frame_heading;

      // send feedback
      panorama::PanoramaFeedback feedback;
      feedback.frames_captured = frame++;
      as.publishFeedback(feedback);

      if (!continuous_capture) {
        complete = frame > number_of_frames;
      } else {
        complete = total_turn > sgn(spin_speed)*2.0*M_PI;
      }
    } else {
      ROS_DEBUG("[panorama] capture_frame is false");
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
