#ifndef CSQMI_EXPLORATION_EXPLORATION_MANAGER_H_
#define CSQMI_EXPLORATION_EXPLORATION_MANAGER_H_

#include <grid_mapping/angle_grid.h>
#include <csqmi_planning/csqmi_planning.h>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/Header.h>

#include <actionlib/client/simple_action_client.h>
#include <panorama/PanoramaAction.h>
#include <scarab_msgs/MoveAction.h>
#include <geometry_msgs/PoseStamped.h>

#include <string.h>
#include <vector>

namespace csqmi_exploration {

class ExplorationManager
{
  protected:

    ros::NodeHandle nh, pnh;
    ros::Time heartbeat;

    // params
    int robot_id, pan_count, number_of_robots;
    volatile int shared_goals_count;
    volatile bool received_new_map, navigation_succeeded, navigation_in_progress;
    bool leader;
    double scan_range_min, scan_range_max, grid_res, pan_goal_tol;
    std::string tf_prefix, pan_file, pan_server_name, nav_server_name, world_frame;
    std::string robot_frame, goal_method;

    // internal map
    grid_mapping::AngleGrid ang_grid;

    // coordination variables
    ros::Publisher viz_map_pub, pan_grid_pub, pan_pose_pub, skel_pub, goal_pose_pub;
    ros::Publisher frontier_grid_pub;
    std::vector<ros::Subscriber> coord_subs;
    ros::Subscriber activate_sub; // allow high level control of exploration
    std::vector<grid_mapping::Point> pan_locations, goal_locations;

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
    void goalPoseCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void activateCB(const std_msgs::Header::ConstPtr& msg);

    bool capturePanorama();
    bool getWorldTrans(const std::string, geometry_msgs::TransformStamped&) const;
    std::vector<std::vector<grid_mapping::Point>> findFrontiers();
    bool frontierGoal(grid_mapping::Point&);
    bool csqmiGoal(grid_mapping::Point&);

  public:

    ExplorationManager(ros::NodeHandle, ros::NodeHandle);

    void explorationLoop();

};

} // namespace csqmi_exploration

#endif
