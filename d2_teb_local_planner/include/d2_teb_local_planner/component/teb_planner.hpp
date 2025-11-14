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
    explicit TebMotionStandaloneComponent(const rclcpp::NodeOptions & options)
    : Node("teb_motion_standalone", options)
    {
        // Config
        cfg_ = std::make_shared<TebConfig>();
        cfg_->node_name = this->get_name();
        
        loadParameters();
        
        // Dynamic parameters
        dyn_params_handler_ = this->add_on_set_parameters_callback(
            std::bind(&TebMotionStandaloneComponent::parametersCallback, this,
                    std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Loaded parameters from parameter server");
        initializeComponents();
        RCLCPP_INFO(this->get_logger(), "Initialized TEB components");

        // Publishers
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_teb", 10);

        // Subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::SensorDataQoS(),
            std::bind(&TebMotionStandaloneComponent::odomCB, this, std::placeholders::_1));
        
        obstacle_sub_ = this->create_subscription<d2_costmap_converter_msgs::msg::ObstacleArrayMsg>(
            "/obstacles", rclcpp::SensorDataQoS(),
            std::bind(&TebMotionStandaloneComponent::obstacleCB, this, std::placeholders::_1));
        
        // ようは waypointから作った global_plan を受け取る
        global_plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/wp_global_plan", 10,
            std::bind(&TebMotionStandaloneComponent::globalPlanCB, this, std::placeholders::_1));

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / control_rate_)),
            std::bind(&TebMotionStandaloneComponent::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "TEB Motion Standalone Component initialized");
    }

private:
    void loadParameters()
    {
        // Trajectory parameters
        this->declare_parameter("teb_autosize", cfg_->trajectory.teb_autosize);
        this->declare_parameter("dt_ref", cfg_->trajectory.dt_ref);
        this->declare_parameter("dt_hysteresis", cfg_->trajectory.dt_hysteresis);
        this->declare_parameter("min_samples", cfg_->trajectory.min_samples);
        this->declare_parameter("max_samples", cfg_->trajectory.max_samples);
        this->declare_parameter("global_plan_overwrite_orientation", cfg_->trajectory.global_plan_overwrite_orientation);
        this->declare_parameter("allow_init_with_backwards_motion", cfg_->trajectory.allow_init_with_backwards_motion);
        this->declare_parameter("global_plan_viapoint_sep", cfg_->trajectory.global_plan_viapoint_sep);
        this->declare_parameter("via_points_ordered", cfg_->trajectory.via_points_ordered);
        this->declare_parameter("max_global_plan_lookahead_dist", cfg_->trajectory.max_global_plan_lookahead_dist);
        this->declare_parameter("exact_arc_length", cfg_->trajectory.exact_arc_length);
        this->declare_parameter("force_reinit_new_goal_dist", cfg_->trajectory.force_reinit_new_goal_dist);
        this->declare_parameter("force_reinit_new_goal_angular", cfg_->trajectory.force_reinit_new_goal_angular);
        this->declare_parameter("feasibility_check_no_poses", cfg_->trajectory.feasibility_check_no_poses);
        this->declare_parameter("publish_feedback", cfg_->trajectory.publish_feedback);
        this->declare_parameter("control_look_ahead_poses", cfg_->trajectory.control_look_ahead_poses);
        this->declare_parameter("control_rate", 10.0);

        // Robot parameters
        this->declare_parameter("max_vel_x", cfg_->robot.max_vel_x);
        this->declare_parameter("max_vel_x_backwards", cfg_->robot.max_vel_x_backwards);
        this->declare_parameter("max_vel_y", cfg_->robot.max_vel_y);
        this->declare_parameter("max_vel_theta", cfg_->robot.max_vel_theta);
        this->declare_parameter("acc_lim_x", cfg_->robot.acc_lim_x);
        this->declare_parameter("acc_lim_y", cfg_->robot.acc_lim_y);
        this->declare_parameter("acc_lim_theta", cfg_->robot.acc_lim_theta);
        this->declare_parameter("min_turning_radius", cfg_->robot.min_turning_radius);
        this->declare_parameter("wheelbase", cfg_->robot.wheelbase);
        this->declare_parameter("cmd_angle_instead_rotvel", cfg_->robot.cmd_angle_instead_rotvel);

        // Goal tolerance
        this->declare_parameter("xy_goal_tolerance", cfg_->goal_tolerance.xy_goal_tolerance);
        this->declare_parameter("yaw_goal_tolerance", cfg_->goal_tolerance.yaw_goal_tolerance);
        this->declare_parameter("free_goal_vel", cfg_->goal_tolerance.free_goal_vel);

        // Obstacles
        this->declare_parameter("min_obstacle_dist", cfg_->obstacles.min_obstacle_dist);
        this->declare_parameter("inflation_dist", cfg_->obstacles.inflation_dist);
        this->declare_parameter("dynamic_obstacle_inflation_dist", cfg_->obstacles.dynamic_obstacle_inflation_dist);
        this->declare_parameter("include_dynamic_obstacles", cfg_->obstacles.include_dynamic_obstacles);
        this->declare_parameter("include_costmap_obstacles", cfg_->obstacles.include_costmap_obstacles);
        this->declare_parameter("costmap_obstacles_behind_robot_dist", cfg_->obstacles.costmap_obstacles_behind_robot_dist);
        this->declare_parameter("obstacle_poses_affected", cfg_->obstacles.obstacle_poses_affected);
        this->declare_parameter("legacy_obstacle_association", cfg_->obstacles.legacy_obstacle_association);
        this->declare_parameter("obstacle_association_force_inclusion_factor", cfg_->obstacles.obstacle_association_force_inclusion_factor);
        this->declare_parameter("obstacle_association_cutoff_factor", cfg_->obstacles.obstacle_association_cutoff_factor);

        // Optimization
        this->declare_parameter("no_inner_iterations", cfg_->optim.no_inner_iterations);
        this->declare_parameter("no_outer_iterations", cfg_->optim.no_outer_iterations);
        this->declare_parameter("optimization_activate", cfg_->optim.optimization_activate);
        this->declare_parameter("optimization_verbose", cfg_->optim.optimization_verbose);
        this->declare_parameter("penalty_epsilon", cfg_->optim.penalty_epsilon);
        this->declare_parameter("weight_max_vel_x", cfg_->optim.weight_max_vel_x);
        this->declare_parameter("weight_max_vel_theta", cfg_->optim.weight_max_vel_theta);
        this->declare_parameter("weight_acc_lim_x", cfg_->optim.weight_acc_lim_x);
        this->declare_parameter("weight_acc_lim_theta", cfg_->optim.weight_acc_lim_theta);
        this->declare_parameter("weight_kinematics_nh", cfg_->optim.weight_kinematics_nh);
        this->declare_parameter("weight_kinematics_forward_drive", cfg_->optim.weight_kinematics_forward_drive);
        this->declare_parameter("weight_kinematics_turning_radius", cfg_->optim.weight_kinematics_turning_radius);
        this->declare_parameter("weight_optimaltime", cfg_->optim.weight_optimaltime);
        this->declare_parameter("weight_shortest_path", cfg_->optim.weight_shortest_path);
        this->declare_parameter("weight_obstacle", cfg_->optim.weight_obstacle);
        this->declare_parameter("weight_inflation", cfg_->optim.weight_inflation);
        this->declare_parameter("weight_dynamic_obstacle", cfg_->optim.weight_dynamic_obstacle);
        this->declare_parameter("weight_dynamic_obstacle_inflation", cfg_->optim.weight_dynamic_obstacle_inflation);
        this->declare_parameter("weight_viapoint", cfg_->optim.weight_viapoint);
        this->declare_parameter("weight_adapt_factor", cfg_->optim.weight_adapt_factor);

        // Homotopy Class Planner
        this->declare_parameter("enable_homotopy_class_planning", cfg_->hcp.enable_homotopy_class_planning);
        this->declare_parameter("enable_multithreading", cfg_->hcp.enable_multithreading);
        this->declare_parameter("simple_exploration", cfg_->hcp.simple_exploration);
        this->declare_parameter("max_number_classes", cfg_->hcp.max_number_classes);
        this->declare_parameter("selection_cost_hysteresis", cfg_->hcp.selection_cost_hysteresis);
        this->declare_parameter("selection_prefer_initial_plan", cfg_->hcp.selection_prefer_initial_plan);
        this->declare_parameter("selection_obst_cost_scale", cfg_->hcp.selection_obst_cost_scale);
        this->declare_parameter("selection_viapoint_cost_scale", cfg_->hcp.selection_viapoint_cost_scale);
        this->declare_parameter("selection_alternative_time_cost", cfg_->hcp.selection_alternative_time_cost);
        this->declare_parameter("roadmap_graph_no_samples", cfg_->hcp.roadmap_graph_no_samples);
        this->declare_parameter("roadmap_graph_area_width", cfg_->hcp.roadmap_graph_area_width);
        this->declare_parameter("h_signature_prescaler", cfg_->hcp.h_signature_prescaler);
        this->declare_parameter("h_signature_threshold", cfg_->hcp.h_signature_threshold);
        this->declare_parameter("obstacle_heading_threshold", cfg_->hcp.obstacle_heading_threshold);
        this->declare_parameter("viapoints_all_candidates", cfg_->hcp.viapoints_all_candidates);
        this->declare_parameter("visualize_hc_graph", cfg_->hcp.visualize_hc_graph);

        // Footprint model
        this->declare_parameter("footprint_model.type", std::string("point"));
        this->declare_parameter("footprint_model.radius", 0.3);
        this->declare_parameter("footprint_model.vertices", std::vector<double>{});

        // Recovery
        this->declare_parameter("oscillation_avoidance", cfg_->recovery.oscillation_avoidance);
        this->declare_parameter("oscillation_omega_eps", cfg_->recovery.oscillation_omega_eps);
        this->declare_parameter("oscillation_recovery_min_duration", cfg_->recovery.oscillation_recovery_min_duration);
        this->declare_parameter("oscillation_filter_duration", cfg_->recovery.oscillation_filter_duration);
        this->declare_parameter("divergence_detection_enable", cfg_->recovery.divergence_detection_enable);
        this->declare_parameter("divergence_detection_max_chi_squared", cfg_->recovery.divergence_detection_max_chi_squared);
        
        // Other
        this->declare_parameter("predict_pose", cfg_->other.predict_pose);

        // Get parameters
        cfg_->trajectory.teb_autosize = this->get_parameter("teb_autosize").as_bool();
        cfg_->trajectory.dt_ref = this->get_parameter("dt_ref").as_double();
        cfg_->trajectory.dt_hysteresis = this->get_parameter("dt_hysteresis").as_double();
        cfg_->trajectory.min_samples = this->get_parameter("min_samples").as_int();
        cfg_->trajectory.max_samples = this->get_parameter("max_samples").as_int();
        cfg_->trajectory.global_plan_overwrite_orientation = this->get_parameter("global_plan_overwrite_orientation").as_bool();
        cfg_->trajectory.allow_init_with_backwards_motion = this->get_parameter("allow_init_with_backwards_motion").as_bool();
        cfg_->trajectory.global_plan_viapoint_sep = this->get_parameter("global_plan_viapoint_sep").as_double();
        cfg_->trajectory.via_points_ordered = this->get_parameter("via_points_ordered").as_bool();
        cfg_->trajectory.max_global_plan_lookahead_dist = this->get_parameter("max_global_plan_lookahead_dist").as_double();
        cfg_->trajectory.exact_arc_length = this->get_parameter("exact_arc_length").as_bool();
        cfg_->trajectory.force_reinit_new_goal_dist = this->get_parameter("force_reinit_new_goal_dist").as_double();
        cfg_->trajectory.force_reinit_new_goal_angular = this->get_parameter("force_reinit_new_goal_angular").as_double();
        cfg_->trajectory.feasibility_check_no_poses = this->get_parameter("feasibility_check_no_poses").as_int();
        cfg_->trajectory.publish_feedback = this->get_parameter("publish_feedback").as_bool();
        cfg_->trajectory.control_look_ahead_poses = this->get_parameter("control_look_ahead_poses").as_int();
        // cfg_->trajectory.control_rate = this->get_parameter("control_rate").as_double();

        control_rate_ = this->get_parameter("control_rate").as_double();

        cfg_->robot.max_vel_x = this->get_parameter("max_vel_x").as_double();
        cfg_->robot.max_vel_x_backwards = this->get_parameter("max_vel_x_backwards").as_double();
        cfg_->robot.max_vel_y = this->get_parameter("max_vel_y").as_double();
        cfg_->robot.max_vel_theta = this->get_parameter("max_vel_theta").as_double();
        cfg_->robot.acc_lim_x = this->get_parameter("acc_lim_x").as_double();
        cfg_->robot.acc_lim_y = this->get_parameter("acc_lim_y").as_double();
        cfg_->robot.acc_lim_theta = this->get_parameter("acc_lim_theta").as_double();
        cfg_->robot.min_turning_radius = this->get_parameter("min_turning_radius").as_double();
        cfg_->robot.wheelbase = this->get_parameter("wheelbase").as_double();
        cfg_->robot.cmd_angle_instead_rotvel = this->get_parameter("cmd_angle_instead_rotvel").as_bool();

        cfg_->goal_tolerance.xy_goal_tolerance = this->get_parameter("xy_goal_tolerance").as_double();
        cfg_->goal_tolerance.yaw_goal_tolerance = this->get_parameter("yaw_goal_tolerance").as_double();
        cfg_->goal_tolerance.free_goal_vel = this->get_parameter("free_goal_vel").as_bool();

        cfg_->obstacles.min_obstacle_dist = this->get_parameter("min_obstacle_dist").as_double();
        cfg_->obstacles.inflation_dist = this->get_parameter("inflation_dist").as_double();
        cfg_->obstacles.dynamic_obstacle_inflation_dist = this->get_parameter("dynamic_obstacle_inflation_dist").as_double();
        cfg_->obstacles.include_dynamic_obstacles = this->get_parameter("include_dynamic_obstacles").as_bool();
        cfg_->obstacles.include_costmap_obstacles = this->get_parameter("include_costmap_obstacles").as_bool();
        cfg_->obstacles.costmap_obstacles_behind_robot_dist = this->get_parameter("costmap_obstacles_behind_robot_dist").as_double();
        cfg_->obstacles.obstacle_poses_affected = this->get_parameter("obstacle_poses_affected").as_int();
        cfg_->obstacles.legacy_obstacle_association = this->get_parameter("legacy_obstacle_association").as_bool();
        cfg_->obstacles.obstacle_association_force_inclusion_factor = this->get_parameter("obstacle_association_force_inclusion_factor").as_double();
        cfg_->obstacles.obstacle_association_cutoff_factor = this->get_parameter("obstacle_association_cutoff_factor").as_double();

        cfg_->optim.no_inner_iterations = this->get_parameter("no_inner_iterations").as_int();
        cfg_->optim.no_outer_iterations = this->get_parameter("no_outer_iterations").as_int();
        cfg_->optim.optimization_activate = this->get_parameter("optimization_activate").as_bool();
        cfg_->optim.optimization_verbose = this->get_parameter("optimization_verbose").as_bool();
        cfg_->optim.penalty_epsilon = this->get_parameter("penalty_epsilon").as_double();
        cfg_->optim.weight_max_vel_x = this->get_parameter("weight_max_vel_x").as_double();
        cfg_->optim.weight_max_vel_theta = this->get_parameter("weight_max_vel_theta").as_double();
        cfg_->optim.weight_acc_lim_x = this->get_parameter("weight_acc_lim_x").as_double();
        cfg_->optim.weight_acc_lim_theta = this->get_parameter("weight_acc_lim_theta").as_double();
        cfg_->optim.weight_kinematics_nh = this->get_parameter("weight_kinematics_nh").as_double();
        cfg_->optim.weight_kinematics_forward_drive = this->get_parameter("weight_kinematics_forward_drive").as_double();
        cfg_->optim.weight_kinematics_turning_radius = this->get_parameter("weight_kinematics_turning_radius").as_double();
        cfg_->optim.weight_optimaltime = this->get_parameter("weight_optimaltime").as_double();
        cfg_->optim.weight_shortest_path = this->get_parameter("weight_shortest_path").as_double();
        cfg_->optim.weight_obstacle = this->get_parameter("weight_obstacle").as_double();
        cfg_->optim.weight_inflation = this->get_parameter("weight_inflation").as_double();
        cfg_->optim.weight_dynamic_obstacle = this->get_parameter("weight_dynamic_obstacle").as_double();
        cfg_->optim.weight_dynamic_obstacle_inflation = this->get_parameter("weight_dynamic_obstacle_inflation").as_double();
        cfg_->optim.weight_viapoint = this->get_parameter("weight_viapoint").as_double();
        cfg_->optim.weight_adapt_factor = this->get_parameter("weight_adapt_factor").as_double();

        cfg_->hcp.enable_homotopy_class_planning = this->get_parameter("enable_homotopy_class_planning").as_bool();
        cfg_->hcp.enable_multithreading = this->get_parameter("enable_multithreading").as_bool();
        cfg_->hcp.simple_exploration = this->get_parameter("simple_exploration").as_bool();
        cfg_->hcp.max_number_classes = this->get_parameter("max_number_classes").as_int();
        cfg_->hcp.selection_cost_hysteresis = this->get_parameter("selection_cost_hysteresis").as_double();
        cfg_->hcp.selection_prefer_initial_plan = this->get_parameter("selection_prefer_initial_plan").as_double();
        cfg_->hcp.selection_obst_cost_scale = this->get_parameter("selection_obst_cost_scale").as_double();
        cfg_->hcp.selection_viapoint_cost_scale = this->get_parameter("selection_viapoint_cost_scale").as_double();
        cfg_->hcp.selection_alternative_time_cost = this->get_parameter("selection_alternative_time_cost").as_bool();
        cfg_->hcp.roadmap_graph_no_samples = this->get_parameter("roadmap_graph_no_samples").as_int();
        cfg_->hcp.roadmap_graph_area_width = this->get_parameter("roadmap_graph_area_width").as_double();
        cfg_->hcp.h_signature_prescaler = this->get_parameter("h_signature_prescaler").as_double();
        cfg_->hcp.h_signature_threshold = this->get_parameter("h_signature_threshold").as_double();
        cfg_->hcp.obstacle_heading_threshold = this->get_parameter("obstacle_heading_threshold").as_double();
        cfg_->hcp.viapoints_all_candidates = this->get_parameter("viapoints_all_candidates").as_bool();
        cfg_->hcp.visualize_hc_graph = this->get_parameter("visualize_hc_graph").as_bool();

        cfg_->recovery.oscillation_avoidance = this->get_parameter("oscillation_avoidance").as_bool();
        cfg_->recovery.oscillation_omega_eps = this->get_parameter("oscillation_omega_eps").as_double();
        cfg_->recovery.oscillation_recovery_min_duration = this->get_parameter("oscillation_recovery_min_duration").as_double();
        cfg_->recovery.oscillation_filter_duration = this->get_parameter("oscillation_filter_duration").as_double();
        cfg_->recovery.divergence_detection_enable = this->get_parameter("divergence_detection_enable").as_bool();
        cfg_->recovery.divergence_detection_max_chi_squared = this->get_parameter("divergence_detection_max_chi_squared").as_double();
        
        cfg_->other.predict_pose = this->get_parameter("predict_pose").as_bool();

        // Footprint model
        std::string footprint_type = this->get_parameter("footprint_model.type").as_string();
        if (footprint_type == "circular") {
            double radius = this->get_parameter("footprint_model.radius").as_double();
            cfg_->robot_model = std::make_shared<CircularRobotFootprint>(radius);
        } else if (footprint_type == "polygon") {
            std::vector<double> vertices = this->get_parameter("footprint_model.vertices").as_double_array();
            PointRobotFootprint* point_model = new PointRobotFootprint();
            cfg_->robot_model = RobotFootprintModelPtr(point_model);
        } else {
            cfg_->robot_model = std::make_shared<PointRobotFootprint>();
        }

        cfg_->checkParameters();
    }

    void initializeComponents()
    {
        lifecycle_node_ = std::make_shared<nav2_util::LifecycleNode>(this->get_name() + std::string("_lc"));

        cfg_->declareParameters(lifecycle_node_, lifecycle_node_->get_name());
        cfg_->loadRosParamFromNodeHandle(lifecycle_node_, lifecycle_node_->get_name());
        cfg_dyn_params_handler_ = this->add_on_set_parameters_callback(
            std::bind(&TebConfig::dynamicParametersCallback, cfg_.get(), std::placeholders::_1)
        );

        // Visualization
        visualization_ = std::make_shared<TebVisualization>(lifecycle_node_, *cfg_);
        visualization_->on_configure();
        visualization_->on_activate();
        
        // Planner
        if (cfg_->hcp.enable_homotopy_class_planning) {
            planner_ = std::make_shared<HomotopyClassPlanner>(
                lifecycle_node_, *cfg_, &obstacles_, visualization_, &via_points_
            );
        } else {
            planner_ = std::make_shared<TebOptimalPlanner>(
                lifecycle_node_, *cfg_, &obstacles_, visualization_, &via_points_
            );
        }
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
        if (name == "dt_ref") cfg_->trajectory.dt_ref = param.as_double();
        else if (name == "dt_hysteresis") cfg_->trajectory.dt_hysteresis = param.as_double();
        else if (name == "min_samples") cfg_->trajectory.min_samples = param.as_int();
        else if (name == "max_samples") cfg_->trajectory.max_samples = param.as_int();
        
        // Robot
        else if (name == "max_vel_x") cfg_->robot.max_vel_x = param.as_double();
        else if (name == "max_vel_theta") cfg_->robot.max_vel_theta = param.as_double();
        else if (name == "acc_lim_x") cfg_->robot.acc_lim_x = param.as_double();
        else if (name == "acc_lim_theta") cfg_->robot.acc_lim_theta = param.as_double();
        
        // Obstacles
        else if (name == "min_obstacle_dist") cfg_->obstacles.min_obstacle_dist = param.as_double();
        else if (name == "inflation_dist") cfg_->obstacles.inflation_dist = param.as_double();
        
        // Optimization
        else if (name == "weight_obstacle") cfg_->optim.weight_obstacle = param.as_double();
        else if (name == "weight_viapoint") cfg_->optim.weight_viapoint = param.as_double();
        else if (name == "weight_optimaltime") cfg_->optim.weight_optimaltime = param.as_double();
        }

        return result;
    }

    void odomCB(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        last_odom_stamp_ = rclcpp::Time(msg->header.stamp);
        robot_pose_ = PoseSE2(msg->pose.pose);
        robot_vel_ = msg->twist.twist;
    }

    void obstacleCB(const d2_costmap_converter_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(obstacle_mutex_);
        obstacles_.clear();

        int counter = 0;
        for (const auto& obstacle_msg : msg->obstacles) {
            if (counter >= 500) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                    "Too many obstacles received !!!");
            }
            if (obstacle_msg.polygon.points.size() == 1) {
                if (obstacle_msg.radius > 0) {
                    obstacles_.push_back(std::make_shared<CircularObstacle>(
                        obstacle_msg.polygon.points[0].x,
                        obstacle_msg.polygon.points[0].y,
                        obstacle_msg.radius
                    ));
                    counter++;
                } else {
                    obstacles_.push_back(std::make_shared<PointObstacle>(
                        obstacle_msg.polygon.points[0].x,
                        obstacle_msg.polygon.points[0].y
                    ));
                    counter++;
                }
            } else if (obstacle_msg.polygon.points.size() > 1) {
                PolygonObstacle* poly_obst = new PolygonObstacle();
                for (const auto& pt : obstacle_msg.polygon.points) {
                    poly_obst->pushBackVertex(pt.x, pt.y);
                }
                poly_obst->finalizePolygon();
                obstacles_.push_back(ObstaclePtr(poly_obst));
                counter++;
            }
        }
    }

    void globalPlanCB(const nav_msgs::msg::Path::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(via_mutex_);
        global_plan_ = *msg;
        has_global_plan_ = true;
        via_points_.clear();
        if (msg->poses.size() > 2) {
            for (size_t i = 1; i < msg->poses.size() - 1; ++i) {
                via_points_.emplace_back(msg->poses[i].pose.position.x, msg->poses[i].pose.position.y);
            }
        }
    }

    void controlLoop() {
        if (!has_global_plan_) return;

        const rclcpp::Time now = this->get_clock()->now();

        PoseSE2 robot_pose;
        geometry_msgs::msg::Twist robot_vel;
        nav_msgs::msg::Path global_plan;
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            const double deltay = (now - last_odom_stamp_).seconds();
            if (deltay > 0.5) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                    "No odometry received for %.2f seconds.", deltay);
            }
            if (cfg_->other.predict_pose) {
                robot_pose = PoseSE2(
                    robot_pose_.x() + robot_vel_.linear.x * std::cos(robot_pose_.theta()) * deltay,
                    robot_pose_.y() + robot_vel_.linear.x * std::sin(robot_pose_.theta()) * deltay,
                    robot_pose_.theta() + robot_vel_.angular.z * deltay
                );
            } else {
                robot_pose = robot_pose_;
            }
            robot_vel = robot_vel_;
        }
        {
            std::lock_guard<std::mutex> lock(via_mutex_);
            global_plan = global_plan_;
        }

        // グローバルパスが0の場合はcmd_velを0にして終了
        if (global_plan.poses.empty()) {
            if (!switched_) {
                // 一回だけ停止コマンドを送る (残存防止)
                geometry_msgs::msg::Twist stop_cmd;
                stop_cmd.linear.x = 0.0;
                stop_cmd.linear.y = 0.0;
                stop_cmd.angular.z = 0.0;
                cmd_pub_->publish(stop_cmd);
                switched_ = true;
                RCLCPP_INFO(this->get_logger(), "Global plan is empty. STOP TEB planner.");
            }
            return;
        } else {
            if (switched_) {
                switched_ = false;
                RCLCPP_INFO(this->get_logger(), "Global plan received. RESUME TEB planner.");
            }
        }

        std::vector<geometry_msgs::msg::PoseStamped> initial_plan = global_plan.poses;
        if (initial_plan.empty()) return;

        // 現在のロボット位置を開始地点として設定
        geometry_msgs::msg::PoseStamped start_pose;
        start_pose.header = global_plan.header;
        start_pose.pose.position.x = robot_pose.x();
        start_pose.pose.position.y = robot_pose.y();
        start_pose.pose.position.z = 0.0;
        start_pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), robot_pose.theta()));
        
        // グローバルプランから現在位置より前の部分を削除
        size_t start_index = 0;
        double min_dist = std::numeric_limits<double>::max();
        for (size_t i = 0; i < initial_plan.size(); ++i) {
            PoseSE2 pose(initial_plan[i].pose);
            double dist = (pose.position() - robot_pose.position()).norm();
            if (dist < min_dist) {
                min_dist = dist;
                start_index = i;
            }
        }
        
        // 現在位置を開始地点として挿入
        std::vector<geometry_msgs::msg::PoseStamped> pruned_plan;
        pruned_plan.push_back(start_pose);
        pruned_plan.insert(pruned_plan.end(), 
                        initial_plan.begin() + start_index, 
                        initial_plan.end());

        const std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
        bool success = planner_->plan(pruned_plan, &robot_vel, cfg_->goal_tolerance.free_goal_vel);
        const std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
        const double planning_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        RCLCPP_INFO(this->get_logger(), "Planning time: %.2f ms", planning_time);

        if (!success) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Planning failed!");
            planner_->clearPlanner();
            return;
        }

        // Get velocity command
        double vx = 0, vy = 0, omega = 0;
        if (!planner_->getVelocityCommandMyj(vx, vy, omega, 
                                        // cfg_->trajectory.control_look_ahead_poses)) {
                                        2.0)) { //
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Failed to get velocity command!");
            return;
        }

        // Publish command
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = vx;
        cmd.linear.y = vy;
        cmd.angular.z = omega;
        cmd_pub_->publish(cmd);

        // Publish visualizations
        if (visualization_) {
            visualization_->publishObstacles(obstacles_);
            visualization_->publishViaPoints(via_points_);
        }
        planner_->visualize();
    }

    // variables -------------------------------------------------------------
    double control_rate_ = 10.0;

    std::shared_ptr<TebConfig> cfg_;
    std::shared_ptr<TebVisualization> visualization_;
    PlannerInterfacePtr planner_;
    nav2_util::LifecycleNode::SharedPtr lifecycle_node_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr cfg_dyn_params_handler_;
    
    ObstContainer obstacles_;
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> via_points_;
    nav_msgs::msg::Path global_plan_;
    
    rclcpp::Time last_odom_stamp_;
    PoseSE2 robot_pose_;
    PoseSE2 goal_pose_;
    geometry_msgs::msg::Twist robot_vel_;
    bool has_global_plan_ = false;
    bool switched_ = false;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<d2_costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacle_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_plan_sub_;
    
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    std::mutex odom_mutex_;
    std::mutex goal_mutex_;
    std::mutex obstacle_mutex_;
    std::mutex via_mutex_;
    
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
    };

} // namespace d2_teb_local_planner
