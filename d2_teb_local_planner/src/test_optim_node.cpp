/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017.
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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>

#include "d2_teb_local_planner/teb_local_planner_ros.h"
#include "d2_teb_local_planner/homotopy_class_planner.h"
#include "d2_teb_local_planner/teb_config.h"
#include "d2_teb_local_planner/visualization.h"
#include <d2_costmap_converter_msgs/msg/obstacle_array_msg.hpp>

using namespace d2_teb_local_planner;

class TestOptimNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  TestOptimNode() : rclcpp_lifecycle::LifecycleNode("test_optim_node")
  {
    // initialize();
  }

  void initialize()
  {
    auto self_lc = std::static_pointer_cast<rclcpp_lifecycle::LifecycleNode>(shared_from_this());
    marker_server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>("marker_obstacles", self_lc);

    cfg_ = std::make_shared<TebConfig>();
    cfg_->declareParameters(self_lc, this->get_name());
    cfg_->loadRosParamFromNodeHandle(self_lc, this->get_name());

    dyn_params_handler_ = this->add_on_set_parameters_callback(
        std::bind(&TebConfig::dynamicParametersCallback, cfg_.get(), std::placeholders::_1));

    visualization_ = std::make_shared<TebVisualization>(self_lc, *cfg_);
    visualization_->on_configure();
    visualization_->on_activate();

    // Setup robot shape model
    // robot_model_ = TebLocalPlannerROS::getRobotFootprintFromParamServer(self_lc);
    robot_model_ = cfg_->getRobotFootprintFromParamServer(self_lc);
    cfg_->robot_model = robot_model_;

    // Obstacles
    obst_vector_.push_back(std::make_shared<PointObstacle>(-3, 1));
    obst_vector_.push_back(std::make_shared<PointObstacle>(6, 2));
    obst_vector_.push_back(std::make_shared<PointObstacle>(0, 0.1));
    no_fixed_obstacles_ = obst_vector_.size();

    // Dynamic obstacles
    Eigen::Vector2d vel(0.1, -0.3);
    obst_vector_.at(0)->setCentroidVelocity(vel);
    vel = Eigen::Vector2d(-0.3, -0.2);
    obst_vector_.at(1)->setCentroidVelocity(vel);

    // Setup planner (homotopy class planning or just the local teb planner)
    if (cfg_->hcp.enable_homotopy_class_planning)
    {
      planner_ = std::make_shared<HomotopyClassPlanner>(self_lc, *cfg_, &obst_vector_, visualization_, &via_points_);
    }
    else
    {
      planner_ = std::make_shared<TebOptimalPlanner>(self_lc, *cfg_, &obst_vector_, visualization_, &via_points_);
    }
    // planner_->setRobotFootprintModel(robot_model_);

    // Subscribers
    custom_obst_sub_ = this->create_subscription<d2_costmap_converter_msgs::msg::ObstacleArrayMsg>(
        "obstacles", rclcpp::SystemDefaultsQoS(), std::bind(&TestOptimNode::customObstacleCB, this, std::placeholders::_1));
    via_points_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "via_points", rclcpp::SystemDefaultsQoS(), std::bind(&TestOptimNode::viaPointsCB, this, std::placeholders::_1));
    clicked_points_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point", rclcpp::SystemDefaultsQoS(), std::bind(&TestOptimNode::clickedPointsCB, this, std::placeholders::_1));
        
    // Interactive Marker Server
    marker_server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>("marker_obstacles", self_lc);
    for (unsigned int i = 0; i < no_fixed_obstacles_; ++i)
    {
      auto pobst = std::dynamic_pointer_cast<PointObstacle>(obst_vector_.at(i));
      if (pobst)
      {
        createInteractiveMarker(pobst->x(), pobst->y(), i, cfg_->map_frame);
      }
    }
    marker_server_->applyChanges();

    // Timers
    planning_timer_ = this->create_wall_timer(std::chrono::milliseconds(25), std::bind(&TestOptimNode::planningLoop, this));
    publish_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&TestOptimNode::publishLoop, this));

    RCLCPP_INFO(this->get_logger(), "TestOptimNode initialized.");
  }

private:
  // Callbacks
  void planningLoop()
  {
    planner_->plan(PoseSE2(-4, 0, 0), PoseSE2(4, 0, 0));
  }

  void publishLoop()
  {
    if (planner_)
      planner_->visualize();
    if (visualization_)
    {
      visualization_->publishObstacles(obst_vector_);
      visualization_->publishViaPoints(via_points_);
    }
  }

  void customObstacleCB(const d2_costmap_converter_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
  {
    obst_vector_.resize(no_fixed_obstacles_);
    for (const auto &obs : msg->obstacles)
    {
      if (obs.polygon.points.size() == 1)
      {
        if (obs.radius == 0)
          obst_vector_.push_back(std::make_shared<PointObstacle>(obs.polygon.points.front().x, obs.polygon.points.front().y));
        else
          obst_vector_.push_back(std::make_shared<CircularObstacle>(obs.polygon.points.front().x, obs.polygon.points.front().y, obs.radius));
      }
      else if (obs.polygon.points.size() > 1)
      {
        auto polyobst = new PolygonObstacle;
        for (const auto &pt : obs.polygon.points)
        {
          polyobst->pushBackVertex(pt.x, pt.y);
        }
        polyobst->finalizePolygon();
        obst_vector_.push_back(ObstaclePtr(polyobst));
      }

      if (!obst_vector_.empty())
        obst_vector_.back()->setCentroidVelocity(obs.velocities, obs.orientation);
    }
  }

  void viaPointsCB(const nav_msgs::msg::Path::SharedPtr msg)
  {
    RCLCPP_INFO_ONCE(this->get_logger(), "Via-points received. This message is printed once.");
    via_points_.clear();
    for (const auto &pose : msg->poses)
    {
      via_points_.emplace_back(pose.pose.position.x, pose.pose.position.y);
    }
  }

  void clickedPointsCB(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    via_points_.push_back(Eigen::Vector2d(msg->point.x, msg->point.y));
    RCLCPP_INFO_STREAM(this->get_logger(), "Via-point (" << msg->point.x << "," << msg->point.y << ") added.");
    if (cfg_->optim.weight_viapoint <= 0)
      RCLCPP_WARN(this->get_logger(), "Note, via-points are deactivated, since 'weight_via_point' <= 0");
  }

  void obstacleMarkerCB(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
  {
    std::stringstream ss(feedback->marker_name);
    unsigned int index;
    ss >> index;

    if (index >= no_fixed_obstacles_)
      return;

    auto pobst = static_cast<PointObstacle *>(obst_vector_.at(index).get());
    pobst->position() = Eigen::Vector2d(feedback->pose.position.x, feedback->pose.position.y);
  }

  void createInteractiveMarker(const double &init_x, const double &init_y, unsigned int id, std::string frame)
  {
    visualization_msgs::msg::InteractiveMarker i_marker;
    i_marker.header.frame_id = frame;
    i_marker.header.stamp = this->now();
    i_marker.name = std::to_string(id);
    i_marker.description = "Obstacle";
    i_marker.pose.position.x = init_x;
    i_marker.pose.position.y = init_y;
    i_marker.pose.orientation.w = 1.0;

    visualization_msgs::msg::Marker marker;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.scale.x = marker.scale.y = 0.4;
    marker.scale.z = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    visualization_msgs::msg::InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
    control.markers.push_back(marker);
    control.always_visible = true;
    i_marker.controls.push_back(control);

    marker_server_->insert(i_marker);
    marker_server_->setCallback(i_marker.name, std::bind(&TestOptimNode::obstacleMarkerCB, this, std::placeholders::_1));
  }

  // Member variables
  std::shared_ptr<TebConfig> cfg_;
  std::shared_ptr<PlannerInterface> planner_;
  std::shared_ptr<TebVisualization> visualization_;
  RobotFootprintModelPtr robot_model_;
  std::vector<ObstaclePtr> obst_vector_;
  ViaPointContainer via_points_;
  unsigned int no_fixed_obstacles_{0};

  rclcpp::TimerBase::SharedPtr planning_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  rclcpp::Subscription<d2_costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr custom_obst_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr via_points_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_points_sub_;

  std::shared_ptr<interactive_markers::InteractiveMarkerServer> marker_server_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestOptimNode>();
  node->initialize();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}