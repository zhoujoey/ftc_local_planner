#ifndef FTC_LOCAL_PLANNER_FTC_PLANNER_H_
#define FTC_LOCAL_PLANNER_FTC_PLANNER_H_

#include <nav_msgs/Odometry.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <ftc_local_planner/FTCPlannerConfig.h>
#include <nav_core/base_local_planner.h>
//#include <ftc_local_planner/transform_global_plan.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/costmap_model.h>
#include <math.h>
#include <nav_msgs/Path.h>
#include <angles/angles.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

struct pose3d {
	double x;
  double y;
  double theta;
};

namespace ftc_local_planner
{

  class FTCPlanner : public nav_core::BaseLocalPlanner
  {
  public:
    FTCPlanner();
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
    bool isGoalReached();
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
    void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);
    ~FTCPlanner();
    bool isInitialized() {
      return initialized_;
    }
    bool pruneGlobalPlan(const tf::TransformListener& tf, const tf::Stamped<tf::Pose>& global_pose, 
                        std::vector<geometry_msgs::PoseStamped>& global_plan, double dist_behind_robot=1);
    bool transformGlobalPlan(const tf::TransformListener& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
                            const tf::Stamped<tf::Pose>& global_pose,  const costmap_2d::Costmap2D& costmap,
                            const std::string& global_frame, double max_plan_length, std::vector<geometry_msgs::PoseStamped>& transformed_plan,
                            int* current_goal_idx = NULL, tf::StampedTransform* tf_plan_to_global = NULL) const;

  private:
    void reconfigureCB(FTCPlannerConfig &config, uint32_t level);
    double calculateGlobalPlanAngle(tf::Stamped<tf::Pose> current_pose, const std::vector<geometry_msgs::PoseStamped>& plan, int point);
    tf::TransformListener* tf_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    std::vector<geometry_msgs::PoseStamped> transformed_global_plan_;
    bool first_setPlan_;
    tf::Stamped<tf::Pose> goal_pose_;
    tf::Stamped<tf::Pose> old_goal_pose_;
    bool rotate_to_global_plan_;
    dynamic_reconfigure::Server<FTCPlannerConfig> *dsrv_;
    ftc_local_planner::FTCPlannerConfig default_config_;
    ftc_local_planner::FTCPlannerConfig config_;
    bool goal_reached_;
    bool stand_at_goal_;
    ros::Publisher local_plan_publisher_;

    std::string global_frame_;
    std::string robot_base_frame_;
    geometry_msgs::Pose2D robot_pose_;
    geometry_msgs::Pose2D robot_goal_;
    geometry_msgs::Pose2D sim_pose_;
    base_local_planner::OdometryHelperRos odom_helper_;
    ros::Publisher plan_pub_;
    geometry_msgs::Twist robot_vel_;
    tf::Stamped<tf::Pose> current_pose_;
    bool initialized_;
    costmap_2d::Costmap2D* costmap_;
    double ctl_time_, stop_time_, safe_time_;
    double goal_tolerance_{0.3}, angle_tolerance_{0.1}, look_ahead_dist_{1.0};
    double max_th_{0.6}, max_vel_{0.6}, max_acc_{1.0}, oscllision_angle_{0.2}, sim_time_{0.1};
    double acc_lim_x_{1.0}, safty_padding_{1.0}, model_time_{0.2};
    int frequency_{10};
    double start_time_{0}, over_time_{20.0}, inplace_rotate_vel_{0.2};
    base_local_planner::CostmapModel* world_model_;
    void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
    geometry_msgs::Pose2D cal_sim_pose(geometry_msgs::Pose2D start_pose, double vx, double vth, double dt);
    geometry_msgs::Pose2D tf_to_pose2d(tf::Stamped<tf::Pose> global_pose);
  };
};
#endif
