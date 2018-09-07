#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_datatypes.h>
#include <rosbag/bag.h>
#include <curses.h>

sensor_msgs::Image::ConstPtr img_ptr;
void imageCB(const sensor_msgs::Image::ConstPtr& msg)
{
  img_ptr = msg;
}

geometry_msgs::PoseStamped robot_pose;
void poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  robot_pose = *msg;
}

void linAngDisplacement(const geometry_msgs::PoseStamped& pose1,
                        const geometry_msgs::PoseStamped& pose2,
                        double& lin_disp,
                        double& ang_disp)
{
  double dx = pose1.pose.position.x - pose2.pose.position.x;
  double dy = pose1.pose.position.y - pose2.pose.position.y;
  lin_disp = sqrt(pow(dx, 2.0) + pow(dy, 2.0));

  double heading1 = tf::getYaw(pose1.pose.orientation);
  double heading2 = tf::getYaw(pose2.pose.orientation);
  ang_disp = heading1 - heading2;
  if (ang_disp > 2.0*M_PI)
    ang_disp -= 2.0*M_PI;
  else if (ang_disp < -2.0*M_PI)
    ang_disp += 2.0*M_PI;
  ang_disp = fabs(ang_disp);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_collection");

  ros::NodeHandle nh, pnh("~");
  ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("pose", 10, poseCB);

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber img_sub = it.subscribe("image", 1, imageCB);

  double v_speed, v_inc, w_speed, w_inc, save_linear_inc, save_angular_inc;
  std::string bagfile_save_dir;
  pnh.param("vmax", v_speed, 0.8);
  pnh.param("wmax", w_speed, 1.0);
  pnh.param("vinc", v_inc, 0.1);
  pnh.param("winc", w_inc, 0.2);
  pnh.param("save_linear_inc", save_linear_inc, 0.3);
  pnh.param("save_angular_inc", save_angular_inc, 0.2);
  pnh.param("bagfile_save_dir", bagfile_save_dir, std::string(""));


  std::string full_file_name;
  if (bagfile_save_dir.size() > 0)
    full_file_name = bagfile_save_dir + "/image_capture.bag";
  else
    full_file_name = "image_capture.bag";

  rosbag::Bag bag;
  bag.open(full_file_name.c_str(), rosbag::bagmode::Write);

  initscr();
  clear();
  noecho();
  cbreak();
  timeout(50); // wait 50 ms
  nodelay(stdscr, TRUE);

  int row = 0;
  mvprintw(row++, 0, "Publishing motor commands to %s", cmd_pub.getTopic().c_str());
  mvprintw(row++, 0, "Subscribed to images on %s", img_sub.getTopic().c_str());
  mvprintw(row++, 0, "Saving images to %s", full_file_name.c_str());
  mvprintw(row++, 0, "Use w, a, s, d to drive, space to halt, and q to quit");
  mvprintw(row++, 0, "Toggle image capture with c");
  mvprintw(row++, 0, "");

  int ch, capture_count = 0;
  bool capture_images = false;
  double d_lin, d_ang;
  ros::Rate r(10);
  geometry_msgs::Twist twist;
  geometry_msgs::PoseStamped capture_pose;
  capture_pose.pose.orientation.w = 1.0;
  while (nh.ok()) {
    row = 6;
    mvprintw(row++, 0, "Angular: % 0.2f", twist.angular.z);
    mvprintw(row++, 0, "Linear:  % 0.2f", twist.linear.x);
    mvprintw(row++, 0, "Image capture: %s", capture_images ? "enabled " : "disabled");
    mvprintw(row++, 0, "Images captured: %d", capture_count);
    mvprintw(row++, 0, "");
    mvprintw(row++, 0, "linear displacement:  %0.2f", d_lin);
    mvprintw(row++, 0, "angular displacement: %0.2f", d_ang);
    refresh();

    ch = getch();

    if (ch != ERR) {
      if (ch == 'w') {
        twist.linear.x += v_inc;
      } else if (ch == 's') {
        twist.linear.x -= v_inc;
      } else if (ch == 'a') {
        twist.angular.z += w_inc;
      } else if (ch == 'd') {
        twist.angular.z -= w_inc;
      } else if (ch == ' ') {
        twist.angular.z = 0;
        twist.linear.x = 0;
      } else if (ch == 'q') {
        twist.angular.z = 0;
        twist.linear.x = 0;
        cmd_pub.publish(twist);
        ros::Duration(0.5).sleep();
        break;
      } else if (ch == 'c') {
        capture_images = !capture_images;
      }
    }
    twist.linear.x = std::min(std::max(twist.linear.x, -v_speed), v_speed);
    twist.angular.z = std::min(std::max(twist.angular.z, -w_speed), w_speed);
    cmd_pub.publish(twist);

    // save images if robot has moved enough
    if (capture_images) {
      linAngDisplacement(robot_pose, capture_pose, d_lin, d_ang);
      if (capture_count == 0 || d_lin > save_linear_inc || d_ang > save_angular_inc) {
        if (img_ptr) {
          bag.write("/color", img_ptr->header.stamp, img_ptr);
          capture_pose = robot_pose;
          ++capture_count;
        } else {
          ROS_WARN("[image_collection] no image to save!");
        }
      }
    }

    ros::spinOnce();
    r.sleep();
  }
  endwin();
  bag.close();

  return 0;
}
