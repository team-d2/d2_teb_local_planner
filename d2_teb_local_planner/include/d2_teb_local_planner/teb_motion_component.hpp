#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <d2_costmap_converter_msgs/msg/obstacle_array_msg.hpp>

#include "d2_teb_local_planner/teb_config.h"
#include "d2_teb_local_planner/optimal_planner.h"
#include "d2_teb_local_planner/homotopy_class_planner.h"
#include "d2_teb_local_planner/visualization.h"
#include "d2_teb_local_planner/pose_se2.h"

namespace d2_teb_local_planner
{
class TebMotionComponent : public rclcpp::Node
{
public:
  explicit TebMotionComponent(const rclcpp::NodeOptions & options);

private:
  // コールバック
  void odomCB(const nav_msgs::msg::Odometry::SharedPtr msg);
  void goalCB(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void obstaclesCB(const d2_costmap_converter_msgs::msg::ObstacleArrayMsg::SharedPtr msg);
  void viaPointsCB(const nav_msgs::msg::Path::SharedPtr msg);
  void clickedPointCB(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void controlTimer();

  // 補助
  void buildInitialPlan(std::vector<geometry_msgs::msg::PoseStamped>& plan);
  void publishLocalPlan();
  geometry_msgs::msg::PoseStamped toPoseStamped(const PoseSE2& p) const;

private:
  // 設定
  std::shared_ptr<TebConfig> cfg_;
  // プランナ (どちらか)
  std::shared_ptr<PlannerInterface> planner_;
  TebVisualizationPtr visualization_;

  // 状態
  bool have_goal_{false};
  geometry_msgs::msg::PoseStamped goal_;
  nav_msgs::msg::Odometry::SharedPtr last_odom_;
  ViaPointContainer via_points_;
  ObstContainer obstacles_;
  std::mutex obst_mutex_, via_mutex_, odom_mutex_, goal_mutex_;

  // 出力
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_plan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_plan_pub_;

  // 入力
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<d2_costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr obst_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr via_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_sub_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};
} // namespace d2_teb_local_planner

RCLCPP_COMPONENTS_REGISTER_NODE(d2_teb_local_planner::TebMotionComponent)