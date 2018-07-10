#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <relative_localization/Recalibrate.h>
#include <csqmi_exploration/InitRelLocalization.h>

#include <string.h>
#include <vector>

using relative_localization::Recalibrate;

bool all_true(const std::vector<bool>& cond_vec)
{
  for (bool cond : cond_vec)
    if (!cond)
      return false;
  return true;
}

class InitRelLocalizationSRV
{
  private:
    ros::NodeHandle nh;
    ros::ServiceClient matlab_srv_client;
    ros::ServiceServer srv;
    tf2_ros::StaticTransformBroadcaster static_broadcaster;
    bool call_recalibrate = false;
    int number_of_robots;
    std::vector<bool> initialization_complete;

  public:
    InitRelLocalizationSRV(ros::NodeHandle nh_) :
      nh(nh_) 
    {
      if (!nh.getParam("/number_of_robots", number_of_robots)) {
        ROS_ERROR("InitRelLocalizationSRV(...): failed to fetch "
            "number_of_robots");
        exit(EXIT_FAILURE);
      }
      initialization_complete = std::vector<bool>(number_of_robots, false);

      std::string name = "/relative_transform_server/recalibrate";
      matlab_srv_client = nh.serviceClient<Recalibrate>(name);
      ROS_INFO("Waiting for %s service", name.c_str());
      matlab_srv_client.waitForExistence();
      ROS_INFO("%s service ready", name.c_str());

      srv = nh.advertiseService("init", &InitRelLocalizationSRV::serviceCB, this);
    }

    bool serviceCB(csqmi_exploration::InitRelLocalization::Request&  req,
                   csqmi_exploration::InitRelLocalization::Response& res)
    {
      // make sure robot id makes sense given the number of robots running
      if (req.id > number_of_robots || req.id <= 0) {
        ROS_ERROR("initRelLocalization(...): Invalid req.id of %d, "
            "number_of_robots is %d", req.id, number_of_robots);
        res.status = false;
        return false;
      }

      // don't compute relative localization until all robots are ready
      initialization_complete[req.id-1] = true;  
      ROS_INFO("initRelLocalization(...): Robot %d of %d is ready for relative "
          "localization", req.id, number_of_robots);
      if (!all_true(initialization_complete)) {
        res.status = false;
        return true;
      }

      // request recalibration and publish the subsequent transforms
      relative_localization::Recalibrate tf_msg;
      matlab_srv_client.call(tf_msg);
      if (tf_msg.response.success) {
        static_broadcaster.sendTransform(tf_msg.response.transforms);
        res.status = true;
      } else {
        ROS_WARN("initRelLocalization(...): matlab service call failed");
        res.status = false;
      }

      return true;
    }
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "initialize_relative_localization");
  ros::NodeHandle nh;

  InitRelLocalizationSRV rlsrv(nh);

  ros::spin();

  return 0;
}
