#ifndef PANORAMA_H
#define PANORAMA_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "panorama/convert.hpp"
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_kdl/tf2_kdl.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <panorama/PanoramaAction.h>

#include <thread>
#include <mutex>

namespace panorama {

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, 
        sensor_msgs::Image, nav_msgs::Odometry> SyncPolicy;
typedef message_filters::Synchronizer<SyncPolicy> Sync;

class Panorama
{
  private:
    ros::NodeHandle nh, pnh;
    ros::Publisher vel_pub;

    // syncronize color, depth images with odometry
    message_filters::Subscriber<sensor_msgs::Image> depth_sub, color_sub;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub;
    std::shared_ptr<Sync> sensor_sync;

    // actionlib
    actionlib::SimpleActionServer<panorama::PanoramaAction> as;
    std::string action_server_name;

    // tf2
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener listener;
    std::string world_frame, camera_frame, robot_frame, odom_frame;
    
    double spin_speed, current_heading;
    int number_of_frames;
    std::string save_directory, file_name;

    // sensor data
    nav_msgs::Odometry::ConstPtr odom_ptr;
    sensor_msgs::Image::ConstPtr depth_ptr, color_ptr;
    std::mutex data_mutex;

    std::thread panorama_loop_thread;

    // compute the difference between the input angle and the current
    // heading of the robot, respecting (-pi, pi]
    double headingDifference(double);

    // create and publish a geometry_msgs::Twist velocity command to the 
    // robot with angular velocity about z set to the input
    void sendSpinCommand(double);

    // fetch a transform from the tf tree between the specified frames at 
    // the desired time and store the result in the templated data structure
    // using tf2 to perform necessary conversions
    template <class T>
    void getTrans(std::string, std::string, ros::Time, T&);

  public:
    Panorama(ros::NodeHandle, ros::NodeHandle, std::string);

    void poseCB(const geometry_msgs::Pose2D::ConstPtr&);
    void syncCB(const sensor_msgs::Image::ConstPtr&, 
                const sensor_msgs::Image::ConstPtr&, 
                const nav_msgs::Odometry::ConstPtr&);
    void goalCB();
    void preemptCB();

    // the main panorama loop
    void captureLoop();
};

// fetch the transfrom from src_frame to dest_frame from the tf tree at time t
// and return the result as type T (conversions between types are handled by
// tf2::convert)
template <typename T>
void Panorama::getTrans(std::string dest_frame, std::string src_frame, 
    ros::Time t, T& bt)
{
  geometry_msgs::TransformStamped trans;
  try {
    trans = tf_buffer.lookupTransform(dest_frame, src_frame, t);
  } catch (tf2::TransformException &ex) {
    ROS_ERROR("Panorama::fetchTrans(): error fetching transform from %s "
            "to %s: %s", src_frame.c_str(), dest_frame.c_str(), ex.what());
    panorama::PanoramaResult result;
    result.success = false;
    as.setAborted(result);
    return;
  }
  tf2::convert(trans, bt);
}

} // panorama namespace

#endif
