#include "d2_teb_local_planner/teb_motion_component.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace d2_teb_local_planner
{
TebMotionComponent::TebMotionComponent(const rclcpp::NodeOptions & options)
: Node("teb_motion_component", options)
{
  // Config
  cfg_ = std::make_shared<TebConfig>();

  // LifecycleNode が無いので直接 node_name を設定
  cfg_->node_name = this->get_name();

  // 必要な最低限パラメータのみ宣言
  this->declare_parameter("enable_homotopy_class_planning", cfg_->hcp.enable_homotopy_class_planning);
  this->declare_parameter("footprint_model.type", std::string("point"));
  this->declare_parameter("footprint_model.radius", 0.0);
  this->declare_parameter("max_vel_x", cfg_->robot.max_vel_x);
  this->declare_parameter("max_vel_theta", cfg_->robot.max_vel_theta);
  this->declare_parameter("weight_obstacle", cfg_->optim.weight_obstacle);
  this->declare_parameter("weight_viapoint", cfg_->optim.weight_viapoint);

  // 取得
  this->get_parameter("enable_homotopy_class_planning", cfg_->hcp.enable_homotopy_class_planning);
  this->get_parameter("max_vel_x", cfg_->robot.max_vel_x);
  this->get_parameter("max_vel_theta", cfg_->robot.max_vel_theta);
  this->get_parameter("weight_obstacle", cfg_->optim.weight_obstacle);
  this->get_parameter("weight_viapoint", cfg_->optim.weight_viapoint);

  std::string ftype;
  this->get_parameter("footprint_model.type", ftype);
  if (ftype == "circular")
  {
    double r; this->get_parameter("footprint_model.radius", r);
    cfg_->robot_model = std::make_shared<CircularRobotFootprint>(r);
  }
  else
  {
    cfg_->robot_model = std::make_shared<PointRobotFootprint>();
  }

  // Dynamic parameters
  dyn_params_handler_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter>& params)
    {
      for (auto & p : params)
      {
        if (p.get_name() == "max_vel_x") cfg_->robot.max_vel_x = p.as_double();
        else if (p.get_name() == "max_vel_theta") cfg_->robot.max_vel_theta = p.as_double();
        else if (p.get_name() == "weight_obstacle") cfg_->optim.weight_obstacle = p.as_double();
        else if (p.get_name() == "weight_viapoint") cfg_->optim.weight_viapoint = p.as_double();
      }
      rcl_interfaces::msg::SetParametersResult res; res.successful = true; return res;
    }
  );

  // Visualization (LifecycleNode なし -> nullptr で利用)
  visualization_ = std::make_shared<TebVisualization>(nullptr, *cfg_);
  visualization_->on_configure();
  visualization_->on_activate();

  // Planner
  if (cfg_->hcp.enable_homotopy_class_planning)
    planner_ = std::make_shared<HomotopyClassPlanner>(nullptr, *cfg_, &obstacles_, visualization_, &via_points_);
  else
    planner_ = std::make_shared<TebOptimalPlanner>(nullptr, *cfg_, &obstacles_, visualization_, &via_points_);

  // Publisher
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  local_plan_pub_ = this->create_publisher<nav_msgs::msg::Path>("/local_plan", 1);
  global_plan_pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_plan", 1);

  // Subscriptions
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&TebMotionComponent::odomCB, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 1, std::bind(&TebMotionComponent::goalCB, this, std::placeholders::_1));
  obst_sub_ = this->create_subscription<d2_costmap_converter_msgs::msg::ObstacleArrayMsg>(
      "/obstacles", 1, std::bind(&TebMotionComponent::obstaclesCB, this, std::placeholders::_1));
  via_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/via_points", 1, std::bind(&TebMotionComponent::viaPointsCB, this, std::placeholders::_1));
  clicked_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/clicked_point", 1, std::bind(&TebMotionComponent::clickedPointCB, this, std::placeholders::_1));

  control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&TebMotionComponent::controlTimer, this));

  RCLCPP_INFO(this->get_logger(), "TebMotionComponent started.");
}

void TebMotionComponent::odomCB(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(odom_mutex_);
  last_odom_ = msg;
}

void TebMotionComponent::goalCB(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(goal_mutex_);
  goal_ = *msg;
  have_goal_ = true;
  RCLCPP_INFO(this->get_logger(), "Goal updated");
}

void TebMotionComponent::obstaclesCB(const d2_costmap_converter_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(obst_mutex_);
  obstacles_.clear();
  for (auto & o : msg->obstacles)
  {
    if (o.polygon.points.size()==1)
    {
      if (o.radius > 0)
        obstacles_.push_back(std::make_shared<CircularObstacle>(o.polygon.points[0].x, o.polygon.points[0].y, o.radius));
      else
        obstacles_.push_back(std::make_shared<PointObstacle>(o.polygon.points[0].x, o.polygon.points[0].y));
    }
  }
}

void TebMotionComponent::viaPointsCB(const nav_msgs::msg::Path::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(via_mutex_);
  via_points_.clear();
  for (auto & p : msg->poses)
    via_points_.emplace_back(p.pose.position.x, p.pose.position.y);
}

void TebMotionComponent::clickedPointCB(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(via_mutex_);
  via_points_.emplace_back(msg->point.x, msg->point.y);
}

geometry_msgs::msg::PoseStamped TebMotionComponent::toPoseStamped(const PoseSE2& p) const
{
  geometry_msgs::msg::PoseStamped ps;
  ps.header.stamp = this->now();
  ps.header.frame_id = "odom";
  p.toPoseMsg(ps.pose);
  return ps;
}

void TebMotionComponent::buildInitialPlan(std::vector<geometry_msgs::msg::PoseStamped>& plan)
{
  plan.clear();
  if (!last_odom_ || !have_goal_) return;
  geometry_msgs::msg::PoseStamped start;
  start.header = last_odom_->header;
  start.pose = last_odom_->pose.pose;
  plan.push_back(start);
  plan.push_back(goal_);
}

void TebMotionComponent::controlTimer()
{
  if (!have_goal_) return;
  std::shared_ptr<nav_msgs::msg::Odometry> odom;
  {
    std::lock_guard<std::mutex> lk(odom_mutex_);
    odom = last_odom_;
  }
  if (!odom) return;

  // 初期プラン構築
  std::vector<geometry_msgs::msg::PoseStamped> initial_plan;
  buildInitialPlan(initial_plan);
  if (initial_plan.size()<2) return;

  // plan 実行
  if (!planner_->plan(initial_plan, &odom->twist.twist, cfg_->goal_tolerance.free_goal_vel))
    return;

  // 速度抽出
  double vx=0, vy=0, omega=0;
  if (!planner_->getVelocityCommand(vx, vy, omega, cfg_->trajectory.control_look_ahead_poses))
    return;

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = vx;
  cmd.linear.y = vy;
  cmd.angular.z = omega;
  cmd_pub_->publish(cmd);

  publishLocalPlan();
  visualization_->publishViaPoints(via_points_);
  visualization_->publishObstacles(obstacles_);
  planner_->visualize();
}

void TebMotionComponent::publishLocalPlan()
{
  // TebOptimalPlanner 専用 (ダウンキャスト)
  auto top = std::dynamic_pointer_cast<TebOptimalPlanner>(planner_);
  if (!top) return;
  const TimedElasticBand& teb = top->teb();
  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = "odom";
  for (int i=0; i<teb.sizePoses(); ++i)
  {
    path.poses.push_back(toPoseStamped(teb.Pose(i)));
  }
  local_plan_pub_->publish(path);

  // 簡易 global plan (start-goal)
  if (have_goal_ && !path.poses.empty())
  {
    nav_msgs::msg::Path gp;
    gp.header = path.header;
    gp.poses.push_back(path.poses.front());
    gp.poses.push_back(goal_);
    global_plan_pub_->publish(gp);
  }
}
} // namespace d2_teb_local_planner
