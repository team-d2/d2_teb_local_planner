#pragma once

#include "d2_teb_local_planner/optimal_planner.h"
#include "d2_teb_local_planner/homotopy_class_planner.h"
#include "d2_teb_local_planner/visualization.h"
#include "d2_teb_local_planner/obstacles.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <d2_costmap_converter_msgs/msg/obstacle_array_msg.hpp>

#include <chrono>

namespace d2_teb_local_planner
{

class TebMotionStandaloneComponent : public rclcpp::Node
{
public:
    explicit TebMotionStandaloneComponent(const std::string node_name, const std::string node_namespace, const rclcpp::NodeOptions & options)
    : Node(node_name, node_namespace, options),
      teb_cfg_(this->create_teb_config()),
      odom_timeout_duration_(rclcpp::Duration::from_seconds(this->declare_parameter("odom_timeout", 1.0))),
      lifecycle_node_(this->create_lifecycle_node(*teb_cfg_)),
      visualization_(std::make_shared<TebVisualization>(lifecycle_node_, *teb_cfg_)),
      params_setter_(this->add_on_set_parameters_callback(
        std::bind(&TebMotionStandaloneComponent::parametersCallback, this,
        std::placeholders::_1))),
      cfg_params_setter_(this->add_on_set_parameters_callback(
            std::bind(&TebConfig::dynamicParametersCallback, teb_cfg_.get(), std::placeholders::_1))),
      cmd_vel_pub_(this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_teb", 10)),
        odom_sub_(this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::SensorDataQoS(),
            std::bind(&TebMotionStandaloneComponent::odomCB, this, std::placeholders::_1))),
        obstacle_sub_(this->create_subscription<d2_costmap_converter_msgs::msg::ObstacleArrayMsg>(
            "/obstacles", rclcpp::SensorDataQoS(),
            std::bind(&TebMotionStandaloneComponent::obstacleCB, this, std::placeholders::_1))),
        global_plan_sub_(this->create_subscription<nav_msgs::msg::Path>(
            "/wp_global_plan", 10,
            std::bind(&TebMotionStandaloneComponent::globalPlanCB, this, std::placeholders::_1))),
        planning_timer_(this->create_wall_timer(
            std::chrono::duration<double>(1.0 / this->declare_parameter("planning_rate", 10.0)),
            std::bind(&TebMotionStandaloneComponent::planningLoop, this))),
        control_callback_group_(this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)),
        control_timer_(this->create_wall_timer(
            std::chrono::duration<double>(1.0 / this->declare_parameter("control_rate", 20.0)),
            std::bind(&TebMotionStandaloneComponent::controlLoop, this),
            control_callback_group_))
    {
        visualization_->on_configure();
        visualization_->on_activate();
        RCLCPP_INFO(this->get_logger(), "TEB Motion Standalone Component initialized!!!!!!!");
    }

    explicit TebMotionStandaloneComponent(const std::string node_name, const rclcpp::NodeOptions & options)
    : TebMotionStandaloneComponent(node_name, "", options)
    {
    }

    explicit TebMotionStandaloneComponent(const rclcpp::NodeOptions & options)
    : TebMotionStandaloneComponent("teb_motion_standalone", "", options)
    {
    }

private:
    std::unique_ptr<TebConfig> create_teb_config()
    {
        auto cfg = std::make_unique<TebConfig>();
        cfg->node_name = this->get_name();
        // Load parameters from the parameter server
        // Trajectory parameters
        cfg->trajectory.teb_autosize = this->declare_parameter("teb_autosize", cfg->trajectory.teb_autosize);
        cfg->trajectory.dt_ref = this->declare_parameter("dt_ref", cfg->trajectory.dt_ref);
        cfg->trajectory.dt_hysteresis = this->declare_parameter("dt_hysteresis", cfg->trajectory.dt_hysteresis);
        cfg->trajectory.min_samples = this->declare_parameter("min_samples", cfg->trajectory.min_samples);
        cfg->trajectory.max_samples = this->declare_parameter("max_samples", cfg->trajectory.max_samples);
        cfg->trajectory.global_plan_overwrite_orientation = this->declare_parameter("global_plan_overwrite_orientation", cfg->trajectory.global_plan_overwrite_orientation);
        cfg->trajectory.allow_init_with_backwards_motion = this->declare_parameter("allow_init_with_backwards_motion", cfg->trajectory.allow_init_with_backwards_motion);
        cfg->trajectory.global_plan_viapoint_sep = this->declare_parameter("global_plan_viapoint_sep", cfg->trajectory.global_plan_viapoint_sep);
        cfg->trajectory.via_points_ordered = this->declare_parameter("via_points_ordered", cfg->trajectory.via_points_ordered);
        cfg->trajectory.max_global_plan_lookahead_dist = this->declare_parameter("max_global_plan_lookahead_dist", cfg->trajectory.max_global_plan_lookahead_dist);
        cfg->trajectory.exact_arc_length = this->declare_parameter("exact_arc_length", cfg->trajectory.exact_arc_length);
        cfg->trajectory.force_reinit_new_goal_dist = this->declare_parameter("force_reinit_new_goal_dist", cfg->trajectory.force_reinit_new_goal_dist);
        cfg->trajectory.force_reinit_new_goal_angular = this->declare_parameter("force_reinit_new_goal_angular", cfg->trajectory.force_reinit_new_goal_angular);
        cfg->trajectory.feasibility_check_no_poses = this->declare_parameter("feasibility_check_no_poses", cfg->trajectory.feasibility_check_no_poses);
        cfg->trajectory.publish_feedback = this->declare_parameter("publish_feedback", cfg->trajectory.publish_feedback);
        cfg->trajectory.control_look_ahead_poses = this->declare_parameter("control_look_ahead_poses", cfg->trajectory.control_look_ahead_poses);
        // Robot parameters
        cfg->robot.max_vel_x = this->declare_parameter("max_vel_x", cfg->robot.max_vel_x);
        cfg->robot.max_vel_x_backwards = this->declare_parameter("max_vel_x_backwards", cfg->robot.max_vel_x_backwards);
        cfg->robot.max_vel_y = this->declare_parameter("max_vel_y", cfg->robot.max_vel_y);
        cfg->robot.max_vel_theta = this->declare_parameter("max_vel_theta", cfg->robot.max_vel_theta);
        cfg->robot.acc_lim_x = this->declare_parameter("acc_lim_x", cfg->robot.acc_lim_x);
        cfg->robot.acc_lim_y = this->declare_parameter("acc_lim_y", cfg->robot.acc_lim_y);
        cfg->robot.acc_lim_theta = this->declare_parameter("acc_lim_theta", cfg->robot.acc_lim_theta);
        cfg->robot.min_turning_radius = this->declare_parameter("min_turning_radius", cfg->robot.min_turning_radius);
        cfg->robot.wheelbase = this->declare_parameter("wheelbase", cfg->robot.wheelbase);
        cfg->robot.cmd_angle_instead_rotvel = this->declare_parameter("cmd_angle_instead_rotvel", cfg->robot.cmd_angle_instead_rotvel);
        // Goal tolerance
        cfg->goal_tolerance.xy_goal_tolerance = this->declare_parameter("xy_goal_tolerance", cfg->goal_tolerance.xy_goal_tolerance);
        cfg->goal_tolerance.yaw_goal_tolerance = this->declare_parameter("yaw_goal_tolerance", cfg->goal_tolerance.yaw_goal_tolerance);
        cfg->goal_tolerance.free_goal_vel = this->declare_parameter("free_goal_vel", cfg->goal_tolerance.free_goal_vel);
        // Obstacles
        cfg->obstacles.min_obstacle_dist = this->declare_parameter("min_obstacle_dist", cfg->obstacles.min_obstacle_dist);
        cfg->obstacles.inflation_dist = this->declare_parameter("inflation_dist", cfg->obstacles.inflation_dist);
        cfg->obstacles.dynamic_obstacle_inflation_dist = this->declare_parameter("dynamic_obstacle_inflation_dist", cfg->obstacles.dynamic_obstacle_inflation_dist);
        cfg->obstacles.include_dynamic_obstacles = this->declare_parameter("include_dynamic_obstacles", cfg->obstacles.include_dynamic_obstacles);
        cfg->obstacles.include_costmap_obstacles = this->declare_parameter("include_costmap_obstacles", cfg->obstacles.include_costmap_obstacles);
        cfg->obstacles.costmap_obstacles_behind_robot_dist = this->declare_parameter("costmap_obstacles_behind_robot_dist", cfg->obstacles.costmap_obstacles_behind_robot_dist);
        cfg->obstacles.obstacle_poses_affected = this->declare_parameter("obstacle_poses_affected", cfg->obstacles.obstacle_poses_affected);
        cfg->obstacles.legacy_obstacle_association = this->declare_parameter("legacy_obstacle_association", cfg->obstacles.legacy_obstacle_association);
        cfg->obstacles.obstacle_association_force_inclusion_factor = this->declare_parameter("obstacle_association_force_inclusion_factor", cfg->obstacles.obstacle_association_force_inclusion_factor);
        cfg->obstacles.obstacle_association_cutoff_factor = this->declare_parameter("obstacle_association_cutoff_factor", cfg->obstacles.obstacle_association_cutoff_factor);
        // Optimization
        cfg->optim.no_inner_iterations = this->declare_parameter("no_inner_iterations", cfg->optim.no_inner_iterations);
        cfg->optim.no_outer_iterations = this->declare_parameter("no_outer_iterations", cfg->optim.no_outer_iterations);
        cfg->optim.optimization_activate = this->declare_parameter("optimization_activate", cfg->optim.optimization_activate);
        cfg->optim.optimization_verbose = this->declare_parameter("optimization_verbose", cfg->optim.optimization_verbose);
        cfg->optim.penalty_epsilon = this->declare_parameter("penalty_epsilon", cfg->optim.penalty_epsilon);
        cfg->optim.weight_max_vel_x = this->declare_parameter("weight_max_vel_x", cfg->optim.weight_max_vel_x);
        cfg->optim.weight_max_vel_theta = this->declare_parameter("weight_max_vel_theta", cfg->optim.weight_max_vel_theta);
        cfg->optim.weight_acc_lim_x = this->declare_parameter("weight_acc_lim_x", cfg->optim.weight_acc_lim_x);
        cfg->optim.weight_acc_lim_theta = this->declare_parameter("weight_acc_lim_theta", cfg->optim.weight_acc_lim_theta);
        cfg->optim.weight_kinematics_nh = this->declare_parameter("weight_kinematics_nh", cfg->optim.weight_kinematics_nh);
        cfg->optim.weight_kinematics_forward_drive = this->declare_parameter("weight_kinematics_forward_drive", cfg->optim.weight_kinematics_forward_drive);
        cfg->optim.weight_kinematics_turning_radius = this->declare_parameter("weight_kinematics_turning_radius", cfg->optim.weight_kinematics_turning_radius);
        cfg->optim.weight_optimaltime = this->declare_parameter("weight_optimaltime", cfg->optim.weight_optimaltime);
        cfg->optim.weight_shortest_path = this->declare_parameter("weight_shortest_path", cfg->optim.weight_shortest_path);
        cfg->optim.weight_obstacle = this->declare_parameter("weight_obstacle", cfg->optim.weight_obstacle);
        cfg->optim.weight_inflation = this->declare_parameter("weight_inflation", cfg->optim.weight_inflation);
        cfg->optim.weight_dynamic_obstacle = this->declare_parameter("weight_dynamic_obstacle", cfg->optim.weight_dynamic_obstacle);
        cfg->optim.weight_dynamic_obstacle_inflation = this->declare_parameter("weight_dynamic_obstacle_inflation", cfg->optim.weight_dynamic_obstacle_inflation);
        cfg->optim.weight_viapoint = this->declare_parameter("weight_viapoint", cfg->optim.weight_viapoint);
        cfg->optim.weight_adapt_factor = this->declare_parameter("weight_adapt_factor", cfg->optim.weight_adapt_factor);
        // Homotopy class planner parameters
        cfg->hcp.enable_homotopy_class_planning = this->declare_parameter("hcp.enable_homotopy_class_planning", cfg->hcp.enable_homotopy_class_planning);
        cfg->hcp.enable_multithreading = this->declare_parameter("hcp.enable_multithreading", cfg->hcp.enable_multithreading);
        cfg->hcp.simple_exploration = this->declare_parameter("hcp.simple_exploration", cfg->hcp.simple_exploration);
        cfg->hcp.max_number_classes = this->declare_parameter("hcp.max_number_classes", cfg->hcp.max_number_classes);
        cfg->hcp.selection_cost_hysteresis = this->declare_parameter("hcp.selection_cost_hysteresis", cfg->hcp.selection_cost_hysteresis);
        cfg->hcp.selection_prefer_initial_plan = this->declare_parameter("hcp.selection_prefer_initial_plan", cfg->hcp.selection_prefer_initial_plan);
        cfg->hcp.selection_obst_cost_scale = this->declare_parameter("hcp.selection_obst_cost_scale", cfg->hcp.selection_obst_cost_scale);
        cfg->hcp.selection_viapoint_cost_scale = this->declare_parameter("hcp.selection_viapoint_cost_scale", cfg->hcp.selection_viapoint_cost_scale);
        cfg->hcp.selection_alternative_time_cost = this->declare_parameter("hcp.selection_alternative_time_cost", cfg->hcp.selection_alternative_time_cost);
        cfg->hcp.roadmap_graph_no_samples = this->declare_parameter("hcp.roadmap_graph_no_samples", cfg->hcp.roadmap_graph_no_samples);
        cfg->hcp.roadmap_graph_area_width = this->declare_parameter("hcp.roadmap_graph_area_width", cfg->hcp.roadmap_graph_area_width);
        cfg->hcp.h_signature_prescaler = this->declare_parameter("hcp.h_signature_prescaler", cfg->hcp.h_signature_prescaler);
        cfg->hcp.h_signature_threshold = this->declare_parameter("hcp.h_signature_threshold", cfg->hcp.h_signature_threshold);
        cfg->hcp.obstacle_heading_threshold = this->declare_parameter("hcp.obstacle_heading_threshold", cfg->hcp.obstacle_heading_threshold);
        cfg->hcp.viapoints_all_candidates = this->declare_parameter("hcp.viapoints_all_candidates", cfg->hcp.viapoints_all_candidates);
        cfg->hcp.visualize_hc_graph = this->declare_parameter("hcp.visualize_hc_graph", cfg->hcp.visualize_hc_graph);

        // Footprint model
        const auto footprint_model_type = this->declare_parameter("footprint_model.type", "point");
        if (footprint_model_type == "circular") {
            cfg->robot_model = std::make_shared<CircularRobotFootprint>(this->declare_parameter("footprint_model.radius", 0.3));
        } else if (footprint_model_type == "polygon") {
            // std::vector<double> vertices = this->declare_parameter("footprint_model.vertices", std::vector<double>{});
            cfg->robot_model = std::make_shared<PointRobotFootprint>();
        } else {
            cfg->robot_model = std::make_shared<PointRobotFootprint>();
        }

        // Recoverie
        cfg->recovery.oscillation_v_eps = this->declare_parameter("oscillation_v_eps", cfg->recovery.oscillation_v_eps);
        cfg->recovery.oscillation_omega_eps = this->declare_parameter("oscillation_omega_eps", cfg->recovery.oscillation_omega_eps);
        cfg->recovery.oscillation_recovery_min_duration = this->declare_parameter("oscillation_recovery_min_duration", cfg->recovery.oscillation_recovery_min_duration);
        cfg->recovery.oscillation_filter_duration = this->declare_parameter("oscillation_filter_duration", cfg->recovery.oscillation_filter_duration);
        cfg->recovery.divergence_detection_enable = this->declare_parameter("divergence_detection_enable", cfg->recovery.divergence_detection_enable);
        cfg->recovery.divergence_detection_max_chi_squared = this->declare_parameter("divergence_detection_max_chi_squared", cfg->recovery.divergence_detection_max_chi_squared);

        // Other
        cfg->other.predict_pose = this->declare_parameter("predict_pose", cfg->other.predict_pose);

        return cfg;
    }

    std::shared_ptr<nav2_util::LifecycleNode> create_lifecycle_node(TebConfig & teb_cfg)
    {
        auto lifecycle_node = std::make_shared<nav2_util::LifecycleNode>(this->get_name() + std::string("_lc"), this->get_namespace(), this->get_node_options());
        teb_cfg_->declareParameters(lifecycle_node, lifecycle_node->get_name());
        teb_cfg_->loadRosParamFromNodeHandle(lifecycle_node, lifecycle_node->get_name());
        return lifecycle_node;
    }

    // 動的パラメータ更新
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> & parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto & param : parameters) {
        const std::string & name = param.get_name();
        
        // Trajectory
        if (name == "dt_ref") teb_cfg_->trajectory.dt_ref = param.as_double();
        else if (name == "dt_hysteresis") teb_cfg_->trajectory.dt_hysteresis = param.as_double();
        else if (name == "min_samples") teb_cfg_->trajectory.min_samples = param.as_int();
        else if (name == "max_samples") teb_cfg_->trajectory.max_samples = param.as_int();
        
        // Robot
        else if (name == "max_vel_x") teb_cfg_->robot.max_vel_x = param.as_double();
        else if (name == "max_vel_theta") teb_cfg_->robot.max_vel_theta = param.as_double();
        else if (name == "acc_lim_x") teb_cfg_->robot.acc_lim_x = param.as_double();
        else if (name == "acc_lim_theta") teb_cfg_->robot.acc_lim_theta = param.as_double();
        
        // Obstacles
        else if (name == "min_obstacle_dist") teb_cfg_->obstacles.min_obstacle_dist = param.as_double();
        else if (name == "inflation_dist") teb_cfg_->obstacles.inflation_dist = param.as_double();
        
        // Optimization
        else if (name == "weight_obstacle") teb_cfg_->optim.weight_obstacle = param.as_double();
        else if (name == "weight_viapoint") teb_cfg_->optim.weight_viapoint = param.as_double();
        else if (name == "weight_optimaltime") teb_cfg_->optim.weight_optimaltime = param.as_double();
        }

        return result;
    }

    void odomCB(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_msg_ = std::move(msg);
    }

    void obstacleCB(const d2_costmap_converter_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
    {
        obstacles_.clear();
        obstacles_.reserve(msg->obstacles.size());

        for (const auto& obstacle_msg : msg->obstacles) {
            if (obstacles_.size() >= 500) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                    "Too many obstacles received !!!");
            }
            if (obstacle_msg.polygon.points.size() == 1) {
                if (obstacle_msg.radius > 0) {
                    obstacles_.push_back(std::make_shared<CircularObstacle>(
                        obstacle_msg.polygon.points[0].x,
                        obstacle_msg.polygon.points[0].y,
                        obstacle_msg.radius));
                } else {
                    obstacles_.push_back(std::make_shared<PointObstacle>(
                        obstacle_msg.polygon.points[0].x,
                        obstacle_msg.polygon.points[0].y));
                }
            } else if (obstacle_msg.polygon.points.size() > 1) {
                auto poly_obst = std::make_shared<PolygonObstacle>();
                for (const auto& pt : obstacle_msg.polygon.points) {
                    poly_obst->pushBackVertex(pt.x, pt.y);
                }
                poly_obst->finalizePolygon();
                obstacles_.push_back(std::move(poly_obst));
            }
            else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                    "Received obstacle with no points!");
            }
        }
    }

    void globalPlanCB(nav_msgs::msg::Path::ConstSharedPtr msg)
    {
        if (msg->poses.empty()) {
            global_plan_msg_.reset();
            via_points_.clear();
            return;
        }

        via_points_.clear();
        if (msg->poses.size() > 2) {
            const auto begin_next = std::next(msg->poses.begin());
            const auto end_prev = std::prev(msg->poses.end());
            std::for_each(begin_next, end_prev, [this](const auto& pose){
                via_points_.emplace_back(pose.pose.position.x, pose.pose.position.y);
            });
            return;
        }
        global_plan_msg_ = std::move(msg);
    }

    void planningLoop() {

        if (global_plan_msg_ == nullptr || odom_msg_ == nullptr) {
            planner_.reset();
            std::unique_lock<std::mutex> lock1(cmd_vel_msg_data_map_mutex_);
            if (!cmd_vel_msg_data_map_.empty()) {
                cmd_vel_msg_data_map_.clear();
                cmd_vel_msg_data_map_.emplace(rclcpp::Time(static_cast<std::int64_t>(0), this->get_clock()->get_clock_type()), geometry_msgs::msg::Twist());
            }
            return;
        }
        
        if (planner_ == nullptr) {
            if (teb_cfg_->hcp.enable_homotopy_class_planning) {
                planner_ = std::make_unique<HomotopyClassPlanner>(lifecycle_node_, *teb_cfg_, &obstacles_,
                    visualization_, &via_points_);
            }
            else {
                planner_ = std::make_unique<TebOptimalPlanner>(lifecycle_node_, *teb_cfg_, &obstacles_,
                    visualization_, &via_points_);
            }
        }

        const auto now = this->now();

        // 現在のロボット位置を開始地点として設定
        geometry_msgs::msg::PoseStamped start_pose;
        start_pose.header.stamp = now;
        start_pose.pose = getPoseMsgData(now);
        
        // グローバルプランから現在位置より前の部分を削除
        const auto calc_sqr_dist = [&start_pose](const geometry_msgs::msg::PoseStamped& pose) {
            const auto dx = pose.pose.position.x - start_pose.pose.position.x;
            const auto dy = pose.pose.position.y - start_pose.pose.position.y;
            const auto dz = pose.pose.position.z - start_pose.pose.position.z;
            return dx * dx + dy * dy + dz * dz;
        };

        auto nearest_itr = global_plan_msg_->poses.begin();
        auto nearest_sqr_dist = calc_sqr_dist(*nearest_itr);
        for (auto itr = global_plan_msg_->poses.begin(); itr != global_plan_msg_->poses.end(); ++itr) {
            const auto sqr_dist = calc_sqr_dist(*itr);
            if (sqr_dist < nearest_sqr_dist) {
                nearest_itr = itr;
                nearest_sqr_dist = sqr_dist;
            }
        }
        
        // 現在位置を開始地点として挿入
        std::vector<geometry_msgs::msg::PoseStamped> pruned_plan;
        pruned_plan.push_back(start_pose);
        pruned_plan.insert(pruned_plan.end(), nearest_itr, global_plan_msg_->poses.end());

        const std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
        bool success = planner_->plan(pruned_plan, &odom_msg_->twist.twist, teb_cfg_->goal_tolerance.free_goal_vel);
        const std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
        const double planning_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        RCLCPP_INFO(this->get_logger(), "Planning time: %.2f ms", planning_time);

        if (!success) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Planning failed!");
            planner_.reset();
            return;
        }
        // Get velocity command
        {
            std::lock_guard<std::mutex> lock(cmd_vel_msg_data_map_mutex_);
            cmd_vel_msg_data_map_ = planner_->getCmdVelMsgDataMap(now);
        }

        // Publish visualizations
        if (visualization_) {
            visualization_->publishObstacles(obstacles_);
            visualization_->publishViaPoints(via_points_);
        }
        planner_->visualize();
    }

    void controlLoop() {
        const auto now = this->now();
        
        std::lock_guard<std::mutex> lock(cmd_vel_msg_data_map_mutex_);
        if (cmd_vel_msg_data_map_.empty()) {
            return;
        }

        const auto cmd_vel_msg_itr = cmd_vel_msg_data_map_.upper_bound(now);
        if (cmd_vel_msg_itr == cmd_vel_msg_data_map_.end()) {
            geometry_msgs::msg::Twist cmd_vel_msg;
            cmd_vel_msg.linear.x = 0.0;
            cmd_vel_msg.linear.y = 0.0;
            cmd_vel_msg.angular.z = 0.0;
            cmd_vel_pub_->publish(cmd_vel_msg);
            cmd_vel_msg_data_map_.clear();
            return;
        }

        cmd_vel_pub_->publish(cmd_vel_msg_itr->second);
    }

    geometry_msgs::msg::Pose getPoseMsgData(rclcpp::Time now)
    {
        if (teb_cfg_->other.predict_pose) {
            return odom_msg_->pose.pose;
        }

        const double delay = (now - rclcpp::Time(odom_msg_->header.stamp)).seconds();
        if (delay > 0.5) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "No odometry received for %.2f seconds.", delay);
        }

        const auto vel_angular = Eigen::Vector3d(
            odom_msg_->twist.twist.angular.x,
            odom_msg_->twist.twist.angular.y,
            odom_msg_->twist.twist.angular.z);
        const auto vel_angular_norm = vel_angular.norm();
        const auto q_base = Eigen::Quaterniond(
            odom_msg_->pose.pose.orientation.w,
            odom_msg_->pose.pose.orientation.x,
            odom_msg_->pose.pose.orientation.y,
            odom_msg_->pose.pose.orientation.z);
        const Eigen::Vector3d position_delta = q_base.inverse() * Eigen::Vector3d(
            odom_msg_->twist.twist.linear.x,
            odom_msg_->twist.twist.linear.y,
            odom_msg_->twist.twist.linear.z) * delay;
        geometry_msgs::msg::Pose pose_msg_data;
        pose_msg_data.position.x = odom_msg_->pose.pose.position.x + position_delta.x();
        pose_msg_data.position.y = odom_msg_->pose.pose.position.y + position_delta.y();
        pose_msg_data.position.z = odom_msg_->pose.pose.position.z + position_delta.z();
        if (vel_angular_norm == 0.0) {
            pose_msg_data.orientation = odom_msg_->pose.pose.orientation;
        }
        else {
            const auto q_angle_axis = Eigen::AngleAxisd(
                vel_angular_norm * delay,
                vel_angular / vel_angular_norm);
            const auto q = q_base * q_angle_axis;
            pose_msg_data.orientation.w = q.w();
            pose_msg_data.orientation.x = q.x();
            pose_msg_data.orientation.y = q.y();
            pose_msg_data.orientation.z = q.z();
        }
        return pose_msg_data;
    }

    // variables -------------------------------------------------------------
    using ViaPoints = std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>;

    std::unique_ptr<TebConfig> teb_cfg_;
    rclcpp::Duration odom_timeout_duration_;

    std::shared_ptr<nav2_util::LifecycleNode> lifecycle_node_;
    std::shared_ptr<TebVisualization> visualization_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_setter_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr cfg_params_setter_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<d2_costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacle_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_plan_sub_;
    
    rclcpp::TimerBase::SharedPtr planning_timer_;

    rclcpp::CallbackGroup::SharedPtr control_callback_group_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    ObstContainer obstacles_;
    ViaPoints via_points_;
    nav_msgs::msg::Path::ConstSharedPtr global_plan_msg_;
    nav_msgs::msg::Odometry::ConstSharedPtr odom_msg_;
    std::unique_ptr<PlannerInterface> planner_;
    std::mutex cmd_vel_msg_data_map_mutex_;
    std::map<rclcpp::Time, geometry_msgs::msg::Twist> cmd_vel_msg_data_map_;
};

} // namespace d2_teb_local_planner
