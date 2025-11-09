/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 *********************************************************************/

#include "d2_teb_local_planner/teb_local_planner_ros.h"

#include <boost/algorithm/string.hpp>
#include <string>
#include <stdexcept>

#include <pluginlib/class_list_macros.hpp>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"

#include <nav2_core/exceptions.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <nav_2d_utils/tf_help.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

// costmap converter メッセージ (d2 用)
#include <d2_costmap_converter_msgs/msg/obstacle_array_msg.hpp>

using nav2_util::declare_parameter_if_not_declared;

namespace d2_teb_local_planner
{

TebLocalPlannerROS::TebLocalPlannerROS()
  : costmap_ros_(nullptr),
    tf_(nullptr),
    cfg_(new TebConfig()),
    costmap_model_(nullptr),
    costmap_converter_loader_("d2_costmap_converter", "d2_costmap_converter::BaseCostmapToPolygons"),
    custom_via_points_active_(false),
    no_infeasible_plans_(0),
    last_preferred_rotdir_(RotType::none),
    initialized_(false)
{
}

TebLocalPlannerROS::~TebLocalPlannerROS() = default;

void TebLocalPlannerROS::initialize(nav2_util::LifecycleNode::SharedPtr node)
{
  if (initialized_)
  {
    RCLCPP_INFO(logger_, "teb_local_planner already initialized.");
    return;
  }

  // パラメータ宣言と読み込み
  cfg_->declareParameters(node, name_);
  cfg_->loadRosParamFromNodeHandle(node, name_);

  obstacles_.reserve(500);

  if (cfg_->hcp.enable_homotopy_class_planning)
  {
    planner_ = std::make_shared<HomotopyClassPlanner>(node, *cfg_.get(), &obstacles_, visualization_, &via_points_);
    RCLCPP_INFO(logger_, "Homotopy class planning ENABLED");
  }
  else
  {
    planner_ = std::make_shared<TebOptimalPlanner>(node, *cfg_.get(), &obstacles_, visualization_, &via_points_);
    RCLCPP_INFO(logger_, "Homotopy class planning DISABLED");
  }

  costmap_ = costmap_ros_->getCostmap();

  costmap_model_ = std::make_shared<dwb_critics::ObstacleFootprintCritic>();
  costmap_model_->initialize(node, "costmap_model", name_, costmap_ros_);

  cfg_->map_frame = costmap_ros_->getGlobalFrameID();

  // costmap converter
  if (!cfg_->obstacles.costmap_converter_plugin.empty())
  {
    try
    {
      costmap_converter_ = costmap_converter_loader_.createSharedInstance(cfg_->obstacles.costmap_converter_plugin);
      std::string converter_name = costmap_converter_loader_.getName(cfg_->obstacles.costmap_converter_plugin);
      boost::replace_all(converter_name, "::", "/");

      costmap_converter_node_ = std::make_shared<rclcpp::Node>("costmap_converter_node");
      costmap_converter_->setOdomTopic(cfg_->odom_topic);
      costmap_converter_->initialize(costmap_converter_node_);
      costmap_converter_->setCostmap2D(costmap_);
      const auto rate = std::make_shared<rclcpp::Rate>(static_cast<double>(cfg_->obstacles.costmap_converter_rate));
      costmap_converter_->startWorker(rate, costmap_, cfg_->obstacles.costmap_converter_spin_thread);
    }
    catch (pluginlib::PluginlibException &ex)
    {
      RCLCPP_WARN(logger_,
                  "Costmap converter plugin load failed. Fallback to point obstacles. Error: %s",
                  ex.what());
      costmap_converter_.reset();
    }
  }
  else
  {
    RCLCPP_INFO(logger_, "No costmap converter plugin specified. Using point obstacles.");
  }

  // footprint
  footprint_spec_ = costmap_ros_->getRobotFootprint();
  nav2_costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);

  dyn_params_handler = node->add_on_set_parameters_callback(
      std::bind(&TebConfig::dynamicParametersCallback, cfg_.get(), std::placeholders::_1));

  validateFootprints(cfg_->robot_model->getInscribedRadius(),
                     robot_inscribed_radius_,
                     cfg_->obstacles.min_obstacle_dist);

  custom_obst_sub_ = node->create_subscription<d2_costmap_converter_msgs::msg::ObstacleArrayMsg>(
      "obstacles",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&TebLocalPlannerROS::customObstacleCB, this, std::placeholders::_1));

  via_points_sub_ = node->create_subscription<nav_msgs::msg::Path>(
      "via_points",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&TebLocalPlannerROS::customViaPointsCB, this, std::placeholders::_1));

  double controller_frequency = 5.0;
  node->get_parameter("controller_frequency", controller_frequency);
  failure_detector_.setBufferLength(
      std::round(cfg_->recovery.oscillation_filter_duration * controller_frequency));

  initialized_ = true;
  time_last_infeasible_plan_ = clock_->now();
  time_last_oscillation_ = clock_->now();

  RCLCPP_DEBUG(logger_, "d2_teb_local_planner initialized.");
}

void TebLocalPlannerROS::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  nh_ = parent;
  auto node = nh_.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  costmap_ros_ = costmap_ros;
  tf_ = tf;
  name_ = name;

  initialize(node);
  visualization_ = std::make_shared<TebVisualization>(node, *cfg_);
  visualization_->on_configure();
  planner_->setVisualization(visualization_);
}

void TebLocalPlannerROS::setPlan(const nav_msgs::msg::Path &orig_global_plan)
{
  if (!initialized_)
  {
    RCLCPP_ERROR(logger_, "Planner not initialized");
    return;
  }
  global_plan_.clear();
  global_plan_.reserve(orig_global_plan.poses.size());
  for (const auto &p : orig_global_plan.poses)
  {
    geometry_msgs::msg::PoseStamped out;
    out.pose = p.pose;
    out.header = orig_global_plan.header;
    global_plan_.push_back(out);
  }
}

geometry_msgs::msg::TwistStamped TebLocalPlannerROS::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &velocity,
    nav2_core::GoalChecker *goal_checker)
{
  if (!initialized_)
    throw std::runtime_error("Planner not initialized");

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.header.frame_id = costmap_ros_->getBaseFrameID();

  geometry_msgs::msg::Pose pose_tol;
  geometry_msgs::msg::Twist vel_tol;
  if (goal_checker && goal_checker->getTolerances(pose_tol, vel_tol))
    cfg_->goal_tolerance.xy_goal_tolerance = pose_tol.position.x;

  robot_pose_ = PoseSE2(pose.pose);
  geometry_msgs::msg::PoseStamped robot_pose_msg = pose;
  robot_pose_.toPoseMsg(robot_pose_msg.pose);
  robot_vel_ = velocity;

  pruneGlobalPlan(robot_pose_msg, global_plan_, cfg_->trajectory.global_plan_prune_distance);

  std::vector<geometry_msgs::msg::PoseStamped> transformed_plan;
  int goal_idx = 0;
  geometry_msgs::msg::TransformStamped tf_plan_to_global;

  if (!transformGlobalPlan(global_plan_, robot_pose_msg, *costmap_, cfg_->map_frame,
                           cfg_->trajectory.max_global_plan_lookahead_dist,
                           transformed_plan, &goal_idx, &tf_plan_to_global))
    throw std::runtime_error("Global plan transform failed");

  if (!custom_via_points_active_)
    updateViaPointsContainer(transformed_plan, cfg_->trajectory.global_plan_viapoint_sep);

  configureBackupModes(transformed_plan, goal_idx);

  if (transformed_plan.empty())
    throw std::runtime_error("Empty transformed plan");

  const auto &goal_point = transformed_plan.back();
  robot_goal_.x() = goal_point.pose.position.x;
  robot_goal_.y() = goal_point.pose.position.y;

  if (cfg_->trajectory.global_plan_overwrite_orientation)
  {
    robot_goal_.theta() = estimateLocalGoalOrientation(global_plan_, goal_point, goal_idx, tf_plan_to_global);
    tf2::Quaternion q;
    q.setRPY(0, 0, robot_goal_.theta());
    transformed_plan.back().pose.orientation = tf2::toMsg(q);
  }
  else
    robot_goal_.theta() = tf2::getYaw(goal_point.pose.orientation);

  if (transformed_plan.size() == 1)
    transformed_plan.insert(transformed_plan.begin(), robot_pose_msg);
  transformed_plan.front().pose = robot_pose_msg.pose;

  obstacles_.clear();
  if (costmap_converter_)
    updateObstacleContainerWithCostmapConverter();
  else
    updateObstacleContainerWithCostmap();
  updateObstacleContainerWithCustomObstacles();

  std::lock_guard<std::mutex> cfg_lock(cfg_->configMutex());

  bool success = planner_->plan(transformed_plan, &robot_vel_, cfg_->goal_tolerance.free_goal_vel);
  if (!success)
  {
    planner_->clearPlanner();
    ++no_infeasible_plans_;
    time_last_infeasible_plan_ = clock_->now();
    last_cmd_ = cmd_vel.twist;
    throw std::runtime_error("Planning failed");
  }

  if (planner_->hasDiverged())
  {
    planner_->clearPlanner();
    ++no_infeasible_plans_;
    time_last_infeasible_plan_ = clock_->now();
    last_cmd_ = cmd_vel.twist;
    throw std::runtime_error("Trajectory diverged");
  }

  if (cfg_->robot.is_footprint_dynamic)
  {
    auto updated_fp = costmap_ros_->getRobotFootprint();
    if (updated_fp != footprint_spec_)
    {
      footprint_spec_ = updated_fp;
      nav2_costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);
    }
  }

  if (!planner_->isTrajectoryFeasible(costmap_model_.get(), footprint_spec_,
                                      robot_inscribed_radius_, robot_circumscribed_radius,
                                      cfg_->trajectory.feasibility_check_no_poses,
                                      cfg_->trajectory.feasibility_check_lookahead_distance))
  {
    planner_->clearPlanner();
    ++no_infeasible_plans_;
    time_last_infeasible_plan_ = clock_->now();
    last_cmd_ = cmd_vel.twist;
    throw std::runtime_error("Trajectory not feasible");
  }

  if (!planner_->getVelocityCommand(cmd_vel.twist.linear.x,
                                    cmd_vel.twist.linear.y,
                                    cmd_vel.twist.angular.z,
                                    cfg_->trajectory.control_look_ahead_poses))
  {
    planner_->clearPlanner();
    ++no_infeasible_plans_;
    time_last_infeasible_plan_ = clock_->now();
    last_cmd_ = cmd_vel.twist;
    throw std::runtime_error("Velocity command invalid");
  }

  saturateVelocity(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z,
                   cfg_->robot.max_vel_x, cfg_->robot.max_vel_y,
                   cfg_->robot.max_vel_theta, cfg_->robot.max_vel_x_backwards);

  if (cfg_->robot.cmd_angle_instead_rotvel)
  {
    cmd_vel.twist.angular.z = convertTransRotVelToSteeringAngle(
        cmd_vel.twist.linear.x, cmd_vel.twist.angular.z,
        cfg_->robot.wheelbase, 0.95 * cfg_->robot.min_turning_radius);
    if (!std::isfinite(cmd_vel.twist.angular.z))
    {
      planner_->clearPlanner();
      ++no_infeasible_plans_;
      time_last_infeasible_plan_ = clock_->now();
      last_cmd_ = cmd_vel.twist;
      throw std::runtime_error("Non-finite steering angle");
    }
  }

  no_infeasible_plans_ = 0;
  last_cmd_ = cmd_vel.twist;

  planner_->visualize();
  visualization_->publishObstacles(obstacles_);
  visualization_->publishViaPoints(via_points_);
  visualization_->publishGlobalPlan(global_plan_);

  return cmd_vel;
}

void TebLocalPlannerROS::updateObstacleContainerWithCostmap()
{
  if (!cfg_->obstacles.include_costmap_obstacles) return;

  std::lock_guard<std::recursive_mutex> lock(*costmap_->getMutex());
  Eigen::Vector2d robot_orient = robot_pose_.orientationUnitVec();

  for (unsigned int i = 0; i < costmap_->getSizeInCellsX() - 1; ++i)
  {
    for (unsigned int j = 0; j < costmap_->getSizeInCellsY() - 1; ++j)
    {
      if (costmap_->getCost(i, j) == nav2_costmap_2d::LETHAL_OBSTACLE)
      {
        Eigen::Vector2d obs;
        costmap_->mapToWorld(i, j, obs.coeffRef(0), obs.coeffRef(1));
        Eigen::Vector2d obs_dir = obs - robot_pose_.position();
        if (obs_dir.dot(robot_orient) < 0 &&
            obs_dir.norm() > cfg_->obstacles.costmap_obstacles_behind_robot_dist)
          continue;
        obstacles_.push_back(std::make_shared<PointObstacle>(obs));
      }
    }
  }
}

void TebLocalPlannerROS::updateObstacleContainerWithCostmapConverter()
{
  if (!costmap_converter_) return;

  d2_costmap_converter::ObstacleArrayConstPtr obstacles = costmap_converter_->getObstacles();
  if (!obstacles) return;

  for (const auto &obst : obstacles->obstacles)
  {
    const auto *polygon = &obst.polygon;

    if (polygon->points.size() == 1 && obst.radius > 0)
      obstacles_.push_back(std::make_shared<CircularObstacle>(
          polygon->points[0].x, polygon->points[0].y, obst.radius));
    else if (polygon->points.size() == 1)
      obstacles_.push_back(std::make_shared<PointObstacle>(
          polygon->points[0].x, polygon->points[0].y));
    else if (polygon->points.size() == 2)
      obstacles_.push_back(std::make_shared<LineObstacle>(
          polygon->points[0].x, polygon->points[0].y,
          polygon->points[1].x, polygon->points[1].y));
    else if (polygon->points.size() > 2)
    {
      auto *poly = new PolygonObstacle;
      for (const auto &pt : polygon->points)
        poly->pushBackVertex(pt.x, pt.y);
      poly->finalizePolygon();
      obstacles_.push_back(ObstaclePtr(poly));
    }

    if (!obstacles_.empty())
      obstacles_.back()->setCentroidVelocity(obst.velocities, obst.orientation);
  }
}

void TebLocalPlannerROS::updateObstacleContainerWithCustomObstacles()
{
  std::lock_guard<std::mutex> l(custom_obst_mutex_);
  if (custom_obstacle_msg_.obstacles.empty()) return;

  Eigen::Affine3d obstacle_to_map_eig;
  try
  {
    geometry_msgs::msg::TransformStamped tr = tf_->lookupTransform(
        cfg_->map_frame, tf2::timeFromSec(0),
        custom_obstacle_msg_.header.frame_id, tf2::timeFromSec(0),
        custom_obstacle_msg_.header.frame_id, tf2::durationFromSec(0.5));
    obstacle_to_map_eig = tf2::transformToEigen(tr);
  }
  catch (tf2::TransformException &ex)
  {
    RCLCPP_ERROR(logger_, "%s", ex.what());
    obstacle_to_map_eig.setIdentity();
  }

  for (const auto &o : custom_obstacle_msg_.obstacles)
  {
    const auto &poly = o.polygon.points;
    if (poly.size() == 1 && o.radius > 0)
    {
      Eigen::Vector3d p(poly.front().x, poly.front().y, poly.front().z);
      obstacles_.push_back(std::make_shared<CircularObstacle>(
          (obstacle_to_map_eig * p).head(2), o.radius));
    }
    else if (poly.size() == 1)
    {
      Eigen::Vector3d p(poly.front().x, poly.front().y, poly.front().z);
      obstacles_.push_back(std::make_shared<PointObstacle>(
          (obstacle_to_map_eig * p).head(2)));
    }
    else if (poly.size() == 2)
    {
      Eigen::Vector3d a(poly.front().x, poly.front().y, poly.front().z);
      Eigen::Vector3d b(poly.back().x, poly.back().y, poly.back().z);
      obstacles_.push_back(std::make_shared<LineObstacle>(
          (obstacle_to_map_eig * a).head(2),
          (obstacle_to_map_eig * b).head(2)));
    }
    else if (poly.empty())
    {
      RCLCPP_INFO(logger_, "Empty custom obstacle polygon skipped.");
      continue;
    }
    else
    {
      auto *pob = new PolygonObstacle;
      for (const auto &pt : poly)
      {
        Eigen::Vector3d v(pt.x, pt.y, pt.z);
        pob->pushBackVertex((obstacle_to_map_eig * v).head(2));
      }
      pob->finalizePolygon();
      obstacles_.push_back(ObstaclePtr(pob));
    }

    if (!obstacles_.empty())
      obstacles_.back()->setCentroidVelocity(o.velocities, o.orientation);
  }
}

void TebLocalPlannerROS::customObstacleCB(
    const d2_costmap_converter_msgs::msg::ObstacleArrayMsg::ConstSharedPtr obst_msg)
{
  std::lock_guard<std::mutex> l(custom_obst_mutex_);
  custom_obstacle_msg_ = *obst_msg;
}

void TebLocalPlannerROS::customViaPointsCB(
    const nav_msgs::msg::Path::ConstSharedPtr via_points_msg)
{
  RCLCPP_INFO_ONCE(logger_, "Via-points received (first time).");
  if (cfg_->trajectory.global_plan_viapoint_sep > 0)
  {
    RCLCPP_INFO(logger_, "Via-points already derived from global plan. Ignoring custom ones.");
    custom_via_points_active_ = false;
    return;
  }

  std::lock_guard<std::mutex> l(via_point_mutex_);
  via_points_.clear();
  for (const auto &pose : via_points_msg->poses)
    via_points_.emplace_back(pose.pose.position.x, pose.pose.position.y);
  custom_via_points_active_ = !via_points_.empty();
}

void TebLocalPlannerROS::activate()
{
  visualization_->on_activate();
}

void TebLocalPlannerROS::deactivate()
{
  visualization_->on_deactivate();
}

void TebLocalPlannerROS::cleanup()
{
  visualization_->on_cleanup();
  if (costmap_converter_)
    costmap_converter_->stopWorker();
}

void TebLocalPlannerROS::saturateVelocity(double& vx, double& vy, double& omega,
                                          double max_vel_x, double max_vel_y,
                                          double max_vel_theta, double max_vel_x_backwards) const
{
  // 後退制限
  if (vx < 0.0)
    vx = std::max(vx, -max_vel_x_backwards);
  vx = std::clamp(vx, -max_vel_x, max_vel_x);
  vy = std::clamp(vy, -max_vel_y, max_vel_y);
  omega = std::clamp(omega, -max_vel_theta, max_vel_theta);
}

double TebLocalPlannerROS::convertTransRotVelToSteeringAngle(double v, double omega,
                                                             double wheelbase, double min_turning_radius) const
{
  if (std::fabs(omega) < 1e-6 || std::fabs(v) < 1e-6)
    return 0.0;
  double R = v / omega;
  if (min_turning_radius > 0.0 && std::fabs(R) < min_turning_radius)
    R = (R > 0 ? min_turning_radius : -min_turning_radius);
  return std::atan(wheelbase / R);
}

bool TebLocalPlannerROS::pruneGlobalPlan(const geometry_msgs::msg::PoseStamped& global_pose,
                                         std::vector<geometry_msgs::msg::PoseStamped>& global_plan,
                                         double dist_behind_robot)
{
  if (global_plan.empty()) return false;
  auto it = global_plan.begin();
  while (it != global_plan.end())
  {
    double dx = global_pose.pose.position.x - it->pose.position.x;
    double dy = global_pose.pose.position.y - it->pose.position.y;
    double dist = std::hypot(dx, dy);
    if (dist < dist_behind_robot)
      it = global_plan.erase(it);
    else
      break;
  }
  return true;
}

bool TebLocalPlannerROS::transformGlobalPlan(
    const std::vector<geometry_msgs::msg::PoseStamped>& global_plan,
    const geometry_msgs::msg::PoseStamped& global_pose,
    const nav2_costmap_2d::Costmap2D& /*costmap*/,
    const std::string& global_frame,
    double max_plan_length,
    std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan,
    int* current_goal_idx,
    geometry_msgs::msg::TransformStamped* tf_plan_to_global) const
{
  if (global_plan.empty()) return false;
  transformed_plan.clear();
  transformed_plan.reserve(global_plan.size());

  // 同一フレーム想定の簡易コピー
  double length = 0.0;
  geometry_msgs::msg::PoseStamped prev = global_pose;
  size_t idx = 0;
  for (const auto& p : global_plan)
  {
    transformed_plan.push_back(p);
    if (idx > 0)
      length += std::hypot(p.pose.position.x - prev.pose.position.x,
                           p.pose.position.y - prev.pose.position.y);
    prev = p;
    if (max_plan_length > 0 && length > max_plan_length) break;
    ++idx;
  }
  if (current_goal_idx) *current_goal_idx = static_cast<int>(transformed_plan.size()) - 1;

  if (tf_plan_to_global)
  {
    tf_plan_to_global->header.stamp = clock_->now();
    tf_plan_to_global->header.frame_id = global_frame;
    tf_plan_to_global->child_frame_id = global_frame;
    tf_plan_to_global->transform.rotation.w = 1.0;
  }
  return true;
}

double TebLocalPlannerROS::estimateLocalGoalOrientation(
    const std::vector<geometry_msgs::msg::PoseStamped>& global_plan,
    const geometry_msgs::msg::PoseStamped& local_goal,
    int current_goal_idx,
    const geometry_msgs::msg::TransformStamped& /*tf_plan_to_global*/,
    int moving_average_length) const
{
  if (global_plan.empty()) return tf2::getYaw(local_goal.pose.orientation);
  int start = std::max(0, current_goal_idx - moving_average_length);
  int end = std::min<int>(global_plan.size() - 1, current_goal_idx + moving_average_length);
  if (end <= start) return tf2::getYaw(local_goal.pose.orientation);

  double dx = global_plan[end].pose.position.x - global_plan[start].pose.position.x;
  double dy = global_plan[end].pose.position.y - global_plan[start].pose.position.y;
  return std::atan2(dy, dx);
}

void TebLocalPlannerROS::updateViaPointsContainer(
    const std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan,
    double min_separation)
{
  via_points_.clear();
  if (min_separation <= 0) return;
  double accum = 0.0;
  geometry_msgs::msg::PoseStamped last;
  bool first = true;
  for (const auto& p : transformed_plan)
  {
    if (first)
    {
      via_points_.push_back(Eigen::Vector2d(p.pose.position.x, p.pose.position.y));
      last = p;
      first = false;
    }
    else
    {
      double ds = std::hypot(p.pose.position.x - last.pose.position.x,
                             p.pose.position.y - last.pose.position.y);
      accum += ds;
      if (accum >= min_separation)
      {
        via_points_.push_back(Eigen::Vector2d(p.pose.position.x, p.pose.position.y));
        accum = 0.0;
        last = p;
      }
    }
  }
}

void TebLocalPlannerROS::validateFootprints(double opt_inscribed_radius,
                                            double costmap_inscribed_radius,
                                            double min_obst_dist)
{
  if (opt_inscribed_radius + min_obst_dist < costmap_inscribed_radius)
  {
    RCLCPP_WARN(logger_,
                "validateFootprints(): (opt inscribed + min_obst_dist) < costmap inscribed radius. "
                "Trajectory might collide.");
  }
}

void TebLocalPlannerROS::configureBackupModes(std::vector<geometry_msgs::msg::PoseStamped>& transformed_plan,
                                              int& goal_idx)
{
  // シンプルなフェイルセーフ（空や短すぎる場合は何もしない）
  if (transformed_plan.size() < 2) return;

  // ゴール直前で停止用の向き補正など（簡略化）
  (void)goal_idx;
}

void TebLocalPlannerROS::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (speed_limit < 0) return;
  if (percentage)
  {
    cfg_->robot.max_vel_x = cfg_->robot.base_max_vel_x * speed_limit;
    cfg_->robot.max_vel_x_backwards = cfg_->robot.base_max_vel_x_backwards * speed_limit;
    cfg_->robot.max_vel_theta = cfg_->robot.base_max_vel_theta * speed_limit;
  }
  else
  {
    cfg_->robot.max_vel_x = std::min(cfg_->robot.base_max_vel_x, speed_limit);
    cfg_->robot.max_vel_x_backwards = std::min(cfg_->robot.base_max_vel_x_backwards, speed_limit);
    cfg_->robot.max_vel_theta = std::min(cfg_->robot.base_max_vel_theta, speed_limit);
  }
}

} // namespace d2_teb_local_planner

PLUGINLIB_EXPORT_CLASS(d2_teb_local_planner::TebLocalPlannerROS, nav2_core::Controller)
