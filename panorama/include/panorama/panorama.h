#ifndef PANORAMA_PANORAMA_H_
#define PANORAMA_PANORAMA_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <openni2_xtion/rgbd_sensor.h>

#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_kdl/tf2_kdl.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <panorama/PanoramaAction.h>

#include <thread>
#include <mutex>
#include <functional>

namespace panorama {

class Panorama
{
  private:
    ros::NodeHandle nh, pnh;
    ros::Publisher vel_pub;
    ros::Subscriber pose_sub;

    // syncronize color, depth images with the pose from SLAM
    openni2_xtion::RGBDSensor xtion;
    openni2_xtion::TimeFilter<openni2_xtion::RGBDFramePtr, 
      geometry_msgs::PoseStampedConstPtr> time_filter;

    // actionlib
    actionlib::SimpleActionServer<panorama::PanoramaAction> as;
    std::string action_server_name;

    // tf2
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener listener;
    std::string world_frame, camera_frame, robot_frame;

    bool continuous_capture;
    double spin_speed, current_heading;
    int number_of_frames;
    std::string save_directory, file_name;

    // sensor data
    ros::Time pose_timestamp;
    openni2_xtion::RGBDFramePtr rgbd_ptr;
    std::mutex data_mutex;

    std::thread panorama_loop_thread;

    // compute the difference between the input angle and the current
    // heading of the robot, respecting (-pi, pi]
    double headDiff(double);

    // create and publish a geometry_msgs::Twist velocity command to the 
    // robot with angular velocity about z set to the input
    void sendSpinCommand(double);

    // fetch a transform from the tf tree between the specified frames at 
    // the desired time and store the result in the templated data structure
    // using tf2 to perform necessary conversions
    template <class T>
    bool getTrans(std::string, std::string, ros::Time, T&);

  public:
    Panorama(ros::NodeHandle, ros::NodeHandle, std::string);

    void syncCB(const openni2_xtion::RGBDFramePtr&,
                const geometry_msgs::PoseStamped::ConstPtr&);
    void goalCB();
    void preemptCB();

    // the main panorama loop
    void captureLoop();
};

// fetch the transfrom from src_frame to dest_frame from the tf tree at time t
// and return the result as type T (conversions between types are handled by
// tf2::convert)
template <typename T>
bool Panorama::getTrans(std::string dest_frame, std::string src_frame,
    ros::Time t, T& bt)
{
  geometry_msgs::TransformStamped trans;
  try {
    trans = tf_buffer.lookupTransform(dest_frame, src_frame, t);
  } catch (tf2::TransformException &ex) {
    ROS_WARN("[panorama] error fetching transform from %s to %s: %s",
        src_frame.c_str(), dest_frame.c_str(), ex.what());
    return false;
  }
  tf2::convert(trans, bt);
  return true;
}

} // panorama namespace

#endif
