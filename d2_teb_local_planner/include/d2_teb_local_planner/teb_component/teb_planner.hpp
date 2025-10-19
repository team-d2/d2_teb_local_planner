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
#include <d2_costmap_converter_msgs/msg/obstacle_array_msg.hpp>

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
        
        // 通常のNodeでパラメータを読み込むための代替実装
        loadParameters();
        
        // Dynamic parameters
        dyn_params_handler_ = this->add_on_set_parameters_callback(
            std::bind(&TebMotionStandaloneComponent::parametersCallback, this,
                    std::placeholders::_1));

        initializeComponents();

        // Publishers
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        local_plan_pub_ = this->create_publisher<nav_msgs::msg::Path>("/local_plan", 10);
        global_plan_pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_plan", 10);

        // Subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::SensorDataQoS(),
            std::bind(&TebMotionStandaloneComponent::odomCB, this, std::placeholders::_1));
        
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", rclcpp::SensorDataQoS(),
            std::bind(&TebMotionStandaloneComponent::goalCB, this, std::placeholders::_1));
        
        obstacle_sub_ = this->create_subscription<d2_costmap_converter_msgs::msg::ObstacleArrayMsg>(
            "/obstacles", rclcpp::SensorDataQoS(),
            std::bind(&TebMotionStandaloneComponent::obstacleCB, this, std::placeholders::_1));
        
        via_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/via_points", rclcpp::SensorDataQoS(),
            std::bind(&TebMotionStandaloneComponent::viaPointsCB, this, std::placeholders::_1));

        // Timer
        double control_rate = cfg_->trajectory.control_rate > 0 ? cfg_->trajectory.control_rate : 10.0;
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / control_rate)),
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
        cfg_->trajectory.control_rate = this->get_parameter("control_rate").as_double();

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
        // Visualization
        visualization_ = std::make_shared<TebVisualization>(nullptr, *cfg_);
        visualization_->on_configure();
        visualization_->on_activate();
        
        // Planner
        if (cfg_->hcp.enable_homotopy_class_planning) {
            planner_ = std::make_shared<HomotopyClassPlanner>(
                nullptr, *cfg_, &obstacles_, visualization_, &via_points_
            );
        } else {
            planner_ = std::make_shared<TebOptimalPlanner>(
                nullptr, *cfg_, &obstacles_, visualization_, &via_points_
            );
        }
    }

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
        robot_pose_ = PoseSE2(msg->pose.pose);
        robot_vel_ = msg->twist.twist;
    }

    void goalCB(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        goal_pose_ = PoseSE2(msg->pose);
        has_goal_ = true;
        RCLCPP_INFO(this->get_logger(), "New goal received: (%.2f, %.2f, %.2f)", 
                    goal_pose_.x(), goal_pose_.y(), goal_pose_.theta());
    }

    void obstacleCB(const d2_costmap_converter_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(obstacle_mutex_);
        obstacles_.clear();
        
        for (const auto& obstacle_msg : msg->obstacles) {
            if (obstacle_msg.polygon.points.size() == 1) {
                if (obstacle_msg.radius > 0) {
                    obstacles_.push_back(std::make_shared<CircularObstacle>(
                        obstacle_msg.polygon.points[0].x,
                        obstacle_msg.polygon.points[0].y,
                        obstacle_msg.radius
                    ));
                } else {
                    obstacles_.push_back(std::make_shared<PointObstacle>(
                        obstacle_msg.polygon.points[0].x,
                        obstacle_msg.polygon.points[0].y
                    ));
                }
            } else if (obstacle_msg.polygon.points.size() > 1) {
                PolygonObstacle* poly_obst = new PolygonObstacle();
                for (const auto& pt : obstacle_msg.polygon.points) {
                    poly_obst->pushBackVertex(pt.x, pt.y);
                }
                poly_obst->finalizePolygon();
                obstacles_.push_back(ObstaclePtr(poly_obst));
            }
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Received %zu obstacles", obstacles_.size());
    }

    void viaPointsCB(const nav_msgs::msg::Path::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(via_mutex_);
        via_points_.clear();
        for (const auto& pose : msg->poses) {
            via_points_.emplace_back(pose.pose.position.x, pose.pose.position.y);
        }
    }

    void controlLoop()
    {
        if (!has_goal_) return;

        PoseSE2 robot_pose;
        geometry_msgs::msg::Twist robot_vel;
        PoseSE2 goal_pose;
        
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            robot_pose = robot_pose_;
            robot_vel = robot_vel_;
        }
        {
            std::lock_guard<std::mutex> lock(goal_mutex_);
            goal_pose = goal_pose_;
        }

        // Check if goal reached
        double dist_to_goal = (goal_pose.position() - robot_pose.position()).norm();
        if (dist_to_goal < cfg_->goal_tolerance.xy_goal_tolerance) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Goal reached!");
            geometry_msgs::msg::Twist cmd;
            cmd_pub_->publish(cmd);
            return;
        }

        // Create initial plan
        std::vector<geometry_msgs::msg::PoseStamped> initial_plan;
        geometry_msgs::msg::PoseStamped start_pose, goal_pose_stamped;
        start_pose.header.stamp = this->now();
        start_pose.header.frame_id = "odom";
        robot_pose.toPoseMsg(start_pose.pose);
        
        goal_pose_stamped.header = start_pose.header;
        goal_pose.toPoseMsg(goal_pose_stamped.pose);
        
        initial_plan.push_back(start_pose);
        initial_plan.push_back(goal_pose_stamped);

        // Plan
        bool success = planner_->plan(initial_plan, &robot_vel, 
                                    cfg_->goal_tolerance.free_goal_vel);
        
        if (!success) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Planning failed!");
            return;
        }

        // Get velocity command
        double vx = 0, vy = 0, omega = 0;
        if (!planner_->getVelocityCommand(vx, vy, omega, 
                                        cfg_->trajectory.control_look_ahead_poses)) {
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
        publishLocalPlan();
        publishGlobalPlan(initial_plan);
        visualization_->publishViaPoints(via_points_);
        visualization_->publishObstacles(obstacles_);
        planner_->visualize();
    }

    void publishLocalPlan()
    {
        auto teb_planner = std::dynamic_pointer_cast<TebOptimalPlanner>(planner_);
        if (!teb_planner) return;

        const TimedElasticBand& teb = teb_planner->teb();
        nav_msgs::msg::Path path;
        path.header.stamp = this->now();
        path.header.frame_id = "odom";

        for (int i = 0; i < teb.sizePoses(); ++i) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header = path.header;
            teb.Pose(i).toPoseMsg(pose_stamped.pose);
            path.poses.push_back(pose_stamped);
        }

        local_plan_pub_->publish(path);
    }

    void publishGlobalPlan(const std::vector<geometry_msgs::msg::PoseStamped>& plan)
    {
        nav_msgs::msg::Path path;
        path.header.stamp = this->now();
        path.header.frame_id = "odom";
        path.poses = plan;
        global_plan_pub_->publish(path);
    }

    std::shared_ptr<TebConfig> cfg_;
    std::shared_ptr<TebVisualization> visualization_;
    PlannerInterfacePtr planner_;
    
    ObstContainer obstacles_;
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> via_points_;
    
    PoseSE2 robot_pose_;
    PoseSE2 goal_pose_;
    geometry_msgs::msg::Twist robot_vel_;
    bool has_goal_ = false;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_plan_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_plan_pub_;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<d2_costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacle_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr via_sub_;
    
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    std::mutex odom_mutex_;
    std::mutex goal_mutex_;
    std::mutex obstacle_mutex_;
    std::mutex via_mutex_;
    
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
    };

} // namespace d2_teb_local_planner