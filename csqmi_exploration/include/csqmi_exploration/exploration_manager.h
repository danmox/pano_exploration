#ifndef CSQMI_EXPLORATION_EXPLORATION_MANAGER_H_
#define CSQMI_EXPLORATION_EXPLORATION_MANAGER_H_

#include <grid_mapping/angle_grid.h>

#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>

#include <csqmi_planning/csqmi_planning.h>

#include <actionlib/client/simple_action_client.h>
#include <panorama/PanoramaAction.h>
#include <scarab_msgs/MoveAction.h>
#include <csqmi_exploration/PanGoal.h>

#include <string.h>
#include <vector>

namespace csqmi_exploration {

struct GoalIDPair
{
  grid_mapping::Point point;
  int id;
};

class ExplorationManager
{
  protected:

    ros::NodeHandle nh, pnh;

    // params
    int robot_id, pan_count, number_of_robots;
    volatile int shared_goals_count;
    volatile bool received_new_map, navigation_succeeded;
    bool leader;
    double scan_range_min, scan_range_max;
    std::string tf_prefix, pan_file;

    // internal map
    grid_mapping::AngleGrid ang_grid;

    // coordination variables
    ros::Publisher viz_map_pub, pan_grid_pub, pan_pose_pub, skel_pub, goal_pose_pub;
    std::vector<ros::Subscriber> coord_subs;
    std::vector<grid_mapping::Point> pan_locations;
    std::vector<GoalIDPair> goals;

    // tf2 variables
    tf2_ros::Buffer tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;

    // beam model for CSQMI computation
    std::shared_ptr<CSQMI> objective;

    // action variables
    typedef actionlib::SimpleActionClient<panorama::PanoramaAction> PanAC;
    typedef actionlib::SimpleActionClient<scarab_msgs::MoveAction> MoveAC;
    std::shared_ptr<PanAC> pan_ac;
    std::shared_ptr<MoveAC> move_ac;

    // action callbacks
    void panDoneCB(const actionlib::SimpleClientGoalState& state,
        const panorama::PanoramaResultConstPtr& result);
    void moveDoneCB(const actionlib::SimpleClientGoalState& state,
        const scarab_msgs::MoveResultConstPtr& result);

    // subscriber callbacks
    void mapCB(const grid_mapping::OccupancyGridConstPtr& msg);
    void goalPoseCB(const PanGoalConstPtr& msg);
    void panPoseCB(const PanGoalConstPtr& msg);

    // exploration methods
    bool capturePanorama();

    bool getTrans(std::string, geometry_msgs::TransformStamped&);

  public:

    ExplorationManager(ros::NodeHandle, ros::NodeHandle);

    void explorationLoop();

};

} // namespace csqmi_exploration

#endif
