#include <ros/ros.h>

#include <ftc_local_planner/ftc_planner.h>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(ftc_local_planner::FTCPlanner, nav_core::BaseLocalPlanner)
using namespace std;
namespace ftc_local_planner
{

  FTCPlanner::FTCPlanner() : initialized_(false)
  {
  }

  void FTCPlanner::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
  {
    if (! isInitialized()) {
      ros::NodeHandle private_nh("~/" + name);
      ros::NodeHandle n;
      plan_pub_ = n.advertise<nav_msgs::Path>("prune_plan", 1);
      costmap_ros_ = costmap_ros;
      costmap_ros_->getRobotPose(current_pose_);
      costmap_ = costmap_ros_->getCostmap();
      tf_ = tf;
      global_frame_ = costmap_ros_->getGlobalFrameID();
      robot_base_frame_ = costmap_ros_->getBaseFrameID();
      ctl_time_ = 1.0 / frequency_;
      stop_time_ = max_vel_ / acc_lim_x_;
      odom_helper_.setOdomTopic("odom");
      //Parameter for dynamic reconfigure
      dsrv_ = new dynamic_reconfigure::Server<FTCPlannerConfig>(private_nh);
      dynamic_reconfigure::Server<FTCPlannerConfig>::CallbackType cb = boost::bind(&FTCPlanner::reconfigureCB, this, _1, _2);
      dsrv_->setCallback(cb);
      world_model_ = new base_local_planner::CostmapModel(*costmap_);
      start_time_ = ros::Time::now().toSec();
      initialized_ = true;
      //***
      ROS_INFO("FTCPlanner: Version 2 Init.");
    }
    else{
      ROS_WARN("This planner has already been initialized, doing nothing.");
    }
  }

  void FTCPlanner::reconfigureCB(FTCPlannerConfig &config, uint32_t level)
  {
    goal_tolerance_ = config.goal_tolerance;
    angle_tolerance_ = config.angle_tolerance;
    look_ahead_dist_ = config.look_ahead_dist;
    max_th_ = config.max_th;
    max_vel_ = config.max_vel;
    max_acc_ = config.max_acc;
    oscllision_angle_ = config.oscllision_angle;
    frequency_ = config.frequency;
    safty_padding_ = config.safty_padding;
    over_time_ = config.over_time;
  }

  bool FTCPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
  {
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    global_plan_.clear();
    global_plan_ = plan;
    goal_reached_ = false;
    start_time_ = ros::Time::now().toSec();
    ROS_WARN("local_planner got plan, start count! ");
    return true;
  }

  bool FTCPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
  {
    double run_time = ros::Time::now().toSec();
    if(run_time - start_time_ > over_time_){
      ROS_ERROR("local_planner is over time, quit local plan! ");
      return false;
    }
    safe_time_ = safty_padding_ * stop_time_;
    sim_time_ = ctl_time_ + safe_time_;
    if(!initialized_){
      ROS_ERROR("local_planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = 0;
    goal_reached_ = false;
    // Get robot pose
    tf::Stamped<tf::Pose> robot_pose;
    costmap_ros_->getRobotPose(robot_pose);
    robot_pose_ = tf_to_pose2d(robot_pose);
    // Get robot velocity
    tf::Stamped<tf::Pose> robot_vel_tf;
    odom_helper_.getRobotVel(robot_vel_tf);
    robot_vel_.linear.x = robot_vel_tf.getOrigin().getX();
    robot_vel_.linear.y = robot_vel_tf.getOrigin().getY();
    robot_vel_.angular.z = tf::getYaw(robot_vel_tf.getRotation());

    // prune global plan to cut off parts of the past (spatially before the robot)
    pruneGlobalPlan(*tf_, robot_pose, global_plan_);
    // Transform global plan to the frame of interest (w.r.t. the local costmap)
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    int goal_idx;
    tf::StampedTransform tf_plan_to_global;
    if (!transformGlobalPlan(*tf_, global_plan_, robot_pose, *costmap_, global_frame_, look_ahead_dist_, 
                            transformed_plan, &goal_idx, &tf_plan_to_global))
    {
      ROS_WARN("Could not transform the global plan to the frame of the controller");
      return false;
    }
    if (transformed_plan.empty()){
      ROS_WARN("Transformed plan is empty. Cannot determine a local plan.");
      return false;
    }
    publishPlan(transformed_plan);

    tf::Stamped<tf::Pose> goal_point;
    tf::poseStampedMsgToTF(transformed_plan.back(), goal_point);
    robot_goal_ = tf_to_pose2d(goal_point);
    //check if robot is get goal
    double dist2goal = sqrt( pow(transformed_plan.back().pose.position.x - robot_pose_.x, 2) + pow(transformed_plan.back().pose.position.y - robot_pose_.y, 2) );
    double dist2angle = angles::normalize_angle(robot_pose_.theta - robot_goal_.theta);
    if(dist2goal< goal_tolerance_){
      //ROS_INFO("reach place, rotate to goal angle");
      if (fabs(dist2angle) < angle_tolerance_){
        //ROS_INFO("Goal!!!!");
        goal_reached_ = true;
        return true;
      }
      if (dist2angle>0){
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = -1 * inplace_rotate_vel_;
      }
      if (dist2angle < 0 ){
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = inplace_rotate_vel_;
      }
      double sim_cost = world_model_->footprintCost(robot_pose_.x, robot_pose_.y, robot_pose_.theta + sim_time_ * cmd_vel.angular.z , costmap_ros_->getRobotFootprint(), 0.0, 0.0);
      if(sim_cost < 0){
        ROS_WARN("Inplace Rotate Maybe Collision !!!!.");
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
        goal_reached_ = true;
      }
      return true;      
    }

    //*****************************************//

    int vel_forward = 1;
    double dx = robot_goal_.x - robot_pose_.x;
    double dy = robot_goal_.y - robot_pose_.y;
    double dth = atan2(dy, dx);
    double th_to_goal = dth - robot_pose_.theta;
    //cout<<"look_ahead tf: "<<goal_point.frame_id_<<" goal x: "<<goal_point.getOrigin().getX()<<" goal theta:" <<tf::getYaw(goal_point.getRotation())<<endl;
    //cout<<robot_pose.frame_id_<<" x :"<<robot_pose_.x<<" y:"<<robot_pose_.y<<" theta: "<<robot_pose_.theta<<endl;
    //cout<<"raw_theta: "<<dth<<" final theta:"<<th_to_goal<<endl;
    th_to_goal = angles::normalize_angle(th_to_goal);
    /*
    if ( th_to_goal> 3.14){
      th_to_goal -= 2*3.14;
    }else if (th_to_goal< -3.14){
      th_to_goal += 2*3.14;
    }*/
    if (fabs(th_to_goal) < (3.14/2)){
      vel_forward = 1;
    }
    if (th_to_goal < (-3.14/2)){
      vel_forward = -1;
      th_to_goal -= -3.14;
    }else if (th_to_goal > (3.14/2)){
      vel_forward = -1;
      th_to_goal -= 3.14;
    } 
    if (fabs(th_to_goal) <= oscllision_angle_){
      //vel_forward = 1;
      th_to_goal = 0.0;
    }
    double vth = min(max_th_, fabs(th_to_goal * frequency_));
    //cout<<dth<<"dth- robot_th"<<robot_pose_.theta<<"diff th"<<th_to_goal<<endl;
    double vx =0;
    if (th_to_goal<0){
      vth = -vth;
    }
    if (fabs(th_to_goal) < 2){
      //todo
      vx = pow((2 - fabs(th_to_goal))/2, 3) * max_vel_;
    }else{
      vx = 0;
    }
    cmd_vel.linear.x = vel_forward * vx;
    cmd_vel.angular.z = vth;
    int check_list = 10;
    for (int i = 1; i< check_list; i++){
      sim_pose_ = cal_sim_pose(robot_pose_, cmd_vel.linear.x, cmd_vel.angular.z, sim_time_ / i);
      double sim_cost = world_model_->footprintCost(sim_pose_.x, sim_pose_.y, sim_pose_.theta, costmap_ros_->getRobotFootprint(), 0.0, 0.0);
      if(sim_cost < 0){
        ROS_WARN("Maybe Collision !!!!.");
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
        return true;
      }
    }
    return true;
  }

  geometry_msgs::Pose2D FTCPlanner::cal_sim_pose(geometry_msgs::Pose2D start_pose, double vx, double vth, double dt)
  {
    geometry_msgs::Pose2D sim_pose;
    sim_pose.theta = start_pose.theta + vth * dt;
    sim_pose.x = start_pose.x + vx * dt * cos(start_pose.theta + 0.5 * vth * dt);
    sim_pose.y = start_pose.y + vx * dt * sin(start_pose.theta + 0.5 *vth * dt);
    return sim_pose;
  }

  geometry_msgs::Pose2D FTCPlanner::tf_to_pose2d(tf::Stamped<tf::Pose> global_pose)
  {
    geometry_msgs::Pose2D pose;
    pose.x = global_pose.getOrigin().x();
    pose.y = global_pose.getOrigin().y();
    pose.theta = tf::getYaw(global_pose.getRotation());
    return pose;
  }

  double FTCPlanner::calculateGlobalPlanAngle(tf::Stamped<tf::Pose> current_pose, const std::vector<geometry_msgs::PoseStamped>& plan, int point)
  {
    if(point >= (int)plan.size()){
      point = plan.size()-1;
    }
    double angle = 0;
    double current_th = tf::getYaw(current_pose.getRotation());
    for(int i = 0; i <= point; i++){
      geometry_msgs::PoseStamped x_pose;
      x_pose=transformed_global_plan_.at(point);

      //Calculate the angles between robotpose and global plan point pose
      double angle_to_goal = atan2(x_pose.pose.position.y - current_pose.getOrigin().getY(),
                                    x_pose.pose.position.x - current_pose.getOrigin().getX());
      angle += angle_to_goal;
    }
    //average
    angle = angle/(point+1);
    return angles::shortest_angular_distance(current_th, angle);
  }

  bool FTCPlanner::isGoalReached()
  {
    if(goal_reached_){
      ROS_INFO("FTCPlanner: Goal reached.");
    }
    return goal_reached_;
  }

  bool FTCPlanner::pruneGlobalPlan(const tf::TransformListener& tf, const tf::Stamped<tf::Pose>& global_pose, std::vector<geometry_msgs::PoseStamped>& global_plan, double dist_behind_robot)
  {
    if (global_plan_.empty())
      return true;
    try{
      // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
      tf::StampedTransform global_to_plan_transform;
      tf.lookupTransform(global_plan.front().header.frame_id, global_pose.frame_id_, ros::Time(0), global_to_plan_transform);
      tf::Stamped<tf::Pose> robot;
      robot.setData( global_to_plan_transform * global_pose );
      double dist_thresh_sq = dist_behind_robot*dist_behind_robot;
      // iterate plan until a pose close the robot is found
      std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin();
      std::vector<geometry_msgs::PoseStamped>::iterator erase_end = it;
      while (it != global_plan.end()){
        double dx = robot.getOrigin().x() - it->pose.position.x;
        double dy = robot.getOrigin().y() - it->pose.position.y;
        double dist_sq = dx * dx + dy * dy;
        if (dist_sq < dist_thresh_sq){
          erase_end = it;
          break;
        }
        ++it;
      }
      if (erase_end == global_plan.end())
        return false;
      if (erase_end != global_plan.begin())
        global_plan.erase(global_plan.begin(), erase_end);
    }catch (const tf::TransformException& ex){
      ROS_DEBUG("Cannot prune path since no transform is available: %s\n", ex.what());
      return false;
    }
    return true;
  }

  bool FTCPlanner::transformGlobalPlan(const tf::TransformListener& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
                    const tf::Stamped<tf::Pose>& global_pose, const costmap_2d::Costmap2D& costmap, const std::string& global_frame, double max_plan_length,
                    std::vector<geometry_msgs::PoseStamped>& transformed_plan, int* current_goal_idx, tf::StampedTransform* tf_plan_to_global)const
  {
    // this method is a slightly modified version of base_local_planner/goal_functions.h
    const geometry_msgs::PoseStamped& plan_pose = global_plan[0];
    transformed_plan.clear();
    try {
      if (global_plan.empty()){
        ROS_ERROR("Received plan with zero length");
        *current_goal_idx = 0;
        return false;
      }
      // get plan_to_global_transform from plan frame to global_frame
      tf::StampedTransform plan_to_global_transform;
      tf.waitForTransform(global_frame, ros::Time::now(),
      plan_pose.header.frame_id, plan_pose.header.stamp,
      plan_pose.header.frame_id, ros::Duration(0.5));
      tf.lookupTransform(global_frame, ros::Time(),
      plan_pose.header.frame_id, plan_pose.header.stamp, 
      plan_pose.header.frame_id, plan_to_global_transform);
      //let's get the pose of the robot in the frame of the plan
      tf::Stamped<tf::Pose> robot_pose;
      tf.transformPose(plan_pose.header.frame_id, global_pose, robot_pose);
      //we'll discard points on the plan that are outside the local costmap
      double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                                      costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);
      dist_threshold *= 0.85; // just consider 85% of the costmap size to better incorporate point obstacle that are
                            // located on the border of the local costmap
      int i = 0;
      double sq_dist_threshold = dist_threshold * dist_threshold;
      double sq_dist = 1e10; 
      //we need to loop to a point on the plan that is within a certain distance of the robot
      for(int j=0; j < (int)global_plan.size(); ++j){
        double x_diff = robot_pose.getOrigin().x() - global_plan[j].pose.position.x;
        double y_diff = robot_pose.getOrigin().y() - global_plan[j].pose.position.y;
        double new_sq_dist = x_diff * x_diff + y_diff * y_diff;
        if (new_sq_dist > sq_dist_threshold)
          break;  // force stop if we have reached the costmap border

        if (new_sq_dist < sq_dist){
          sq_dist = new_sq_dist;
          i = j;
        }
      }
      
      tf::Stamped<tf::Pose> tf_pose;
      geometry_msgs::PoseStamped newer_pose;
      double plan_length = 0; // check cumulative Euclidean distance along the plan
      while(i < (int)global_plan.size() && sq_dist <= sq_dist_threshold && (max_plan_length<=0 || plan_length <= max_plan_length)){
        const geometry_msgs::PoseStamped& pose = global_plan[i];
        tf::poseStampedMsgToTF(pose, tf_pose);
        tf_pose.setData(plan_to_global_transform * tf_pose);
        tf_pose.stamp_ = plan_to_global_transform.stamp_;
        tf_pose.frame_id_ = global_frame;
        tf::poseStampedTFToMsg(tf_pose, newer_pose);

        transformed_plan.push_back(newer_pose);

        double x_diff = robot_pose.getOrigin().x() - global_plan[i].pose.position.x;
        double y_diff = robot_pose.getOrigin().y() - global_plan[i].pose.position.y;
        sq_dist = x_diff * x_diff + y_diff * y_diff;
        
        // caclulate distance to previous pose
        if (i>0 && max_plan_length>0){
          double p2d = sqrt( pow(global_plan[i-1].pose.position.x - global_plan[i].pose.position.x, 2) + pow(global_plan[i-1].pose.position.y - global_plan[i].pose.position.y, 2) );
          plan_length += p2d;
        }
        ++i;
      }
      if (transformed_plan.empty()){
        tf::poseStampedMsgToTF(global_plan.back(), tf_pose);
        tf_pose.setData(plan_to_global_transform * tf_pose);
        tf_pose.stamp_ = plan_to_global_transform.stamp_;
        tf_pose.frame_id_ = global_frame;
        tf::poseStampedTFToMsg(tf_pose, newer_pose);
        transformed_plan.push_back(newer_pose);
        // Return the index of the current goal point (inside the distance threshold)
        if (current_goal_idx) *current_goal_idx = int(global_plan.size())-1;
      }else{
        // Return the index of the current goal point (inside the distance threshold)
        if (current_goal_idx) *current_goal_idx = i-1; // subtract 1, since i was increased once before leaving the loop
      }
      // Return the transformation from the global plan to the global planning frame if desired
      if (tf_plan_to_global) *tf_plan_to_global = plan_to_global_transform;
    }catch(tf::LookupException& ex){
      ROS_ERROR("No Transform available Error: %s\n", ex.what());
      return false;
    }catch(tf::ConnectivityException& ex) {
      ROS_ERROR("Connectivity Error: %s\n", ex.what());
      return false;
    }catch(tf::ExtrapolationException& ex) {
      ROS_ERROR("Extrapolation Error: %s\n", ex.what());
      if (global_plan.size() > 0)
        ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

      return false;
    }
    return true;
  }

  void FTCPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) 
  {
      if (!initialized_) {
          ROS_ERROR(
                  "This planner has not been initialized yet, but it is being used, please call initialize() before use");
          return;
      }
      //create a message for the plan
      nav_msgs::Path gui_path;
      gui_path.poses.resize(path.size());
      gui_path.header.frame_id = path[0].header.frame_id;
      gui_path.header.stamp = ros::Time::now();
      // Extract the plan in world co-ordinates, we assume the path is all in the same frame
      for (unsigned int i = 0; i < path.size(); i++) {
          gui_path.poses[i] = path[i];
      }
      plan_pub_.publish(gui_path);
  }

  FTCPlanner::~FTCPlanner()
  {
    delete world_model_;
  }
}
