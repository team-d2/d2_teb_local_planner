#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <d2_costmap_converter_msgs/msg/obstacle_array_msg.hpp>
#include <d2_costmap_converter/costmap_converter_interface.h>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pluginlib/class_loader.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace d2_teb_local_planner
{

class ObstacleFusionComponent : public rclcpp::Node
{
public:
  explicit ObstacleFusionComponent(const rclcpp::NodeOptions & options)
  : Node("obstacle_fusion", options),
    costmap_converter_loader_("d2_costmap_converter", 
                              "d2_costmap_converter::BaseCostmapToPolygons"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    this->declare_parameter("costmap_converter_plugin", 
                            "d2_costmap_converter::CostmapToPolygonsDBSMCCH");
    this->declare_parameter("converter_rate", 5.0);
    this->declare_parameter("costmap_lethal_threshold", 99);
    this->declare_parameter("submap_size_x", 10.0);
    this->declare_parameter("submap_size_y", 10.0);
    this->declare_parameter("target_frame", "map");
    this->declare_parameter("pointcloud_min_distance", 0.5);
    this->declare_parameter("pointcloud_max_distance", 10.0);
    this->declare_parameter("pointcloud_min_height", -0.75);
    this->declare_parameter("pointcloud_max_height", 2.0);
    this->declare_parameter("voxel_leaf_size", 0.1);
    
    std::string plugin_name = this->get_parameter("costmap_converter_plugin").as_string();
    converter_rate_ = this->get_parameter("converter_rate").as_double();
    costmap_threshold_ = this->get_parameter("costmap_lethal_threshold").as_int();
    submap_size_x_ = this->get_parameter("submap_size_x").as_double();
    submap_size_y_ = this->get_parameter("submap_size_y").as_double();
    target_frame_ = this->get_parameter("target_frame").as_string();
    pc_min_dist_ = this->get_parameter("pointcloud_min_distance").as_double();
    pc_max_dist_ = this->get_parameter("pointcloud_max_distance").as_double();
    pc_min_height_ = this->get_parameter("pointcloud_min_height").as_double();
    pc_max_height_ = this->get_parameter("pointcloud_max_height").as_double();
    double voxel_size = this->get_parameter("voxel_leaf_size").as_double();

    voxel_filter_.setLeafSize(voxel_size, voxel_size, voxel_size);

    try {
      costmap_converter_ = costmap_converter_loader_.createSharedInstance(plugin_name);
      auto converter_node = std::make_shared<rclcpp::Node>("costmap_converter_node");
      costmap_converter_->initialize(converter_node);
      RCLCPP_INFO(this->get_logger(), "Loaded costmap converter: %s", plugin_name.c_str());
    } catch (const pluginlib::PluginlibException& ex) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load plugin: %s", ex.what());
      throw;
    }

    obstacle_pub_ = this->create_publisher<d2_costmap_converter_msgs::msg::ObstacleArrayMsg>(
        "/obstacles", 10);
    obstacle_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/obstacle_markers", 10);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::SensorDataQoS(),
      std::bind(&ObstacleFusionComponent::odomCallback, this, std::placeholders::_1));

    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap", rclcpp::QoS(1).transient_local().reliable(),
        std::bind(&ObstacleFusionComponent::costmapCallback, this, std::placeholders::_1));
    
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/points", rclcpp::SensorDataQoS(),
        std::bind(&ObstacleFusionComponent::pointcloudCallback, this, std::placeholders::_1));

    auto period = std::chrono::duration<double>(1.0 / converter_rate_);
    compute_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&ObstacleFusionComponent::computeAndPublish, this));

    RCLCPP_INFO(this->get_logger(), "ObstacleFusionComponent initialized (rate: %.1f Hz)", 
                converter_rate_);
  }

private:
  
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    cached_costmap_ = msg;
    RCLCPP_DEBUG(this->get_logger(), "Received costmap: %dx%d", 
                 msg->info.width, msg->info.height);
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    robot_pose_ = msg->pose.pose;
  }

  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud_raw);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto &pt : cloud_raw->points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) 
        continue;
      
      double dist_xy = std::hypot(pt.x, pt.y);
      if (dist_xy < pc_min_dist_ || dist_xy > pc_max_dist_) 
        continue;
      
      if (pt.z < pc_min_height_ || pt.z > pc_max_height_) 
        continue;
      
      cloud_filtered->points.push_back(pt);
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_filter_.setInputCloud(cloud_filtered);
    voxel_filter_.filter(*cloud_downsampled);
    
    sensor_msgs::msg::PointCloud2 cloud_ros;
    pcl::toROSMsg(*cloud_downsampled, cloud_ros);
    cloud_ros.header.frame_id = msg->header.frame_id;
    cloud_ros.header.stamp = this->now();
    
    sensor_msgs::msg::PointCloud2 cloud_transformed;
    try {
      cloud_transformed = tf_buffer_.transform(
        cloud_ros, target_frame_, tf2::durationFromSec(1.0));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "PointCloud transform failed: %s", ex.what());
      return;
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(cloud_transformed, *cloud_target);
    
    std::lock_guard<std::mutex> lock(pointcloud_mutex_);
    cached_points_.clear();
    cached_points_.reserve(cloud_target->points.size());
    for (const auto &pt : cloud_target->points) {
      cached_points_.push_back(pt);
    }
    
    RCLCPP_DEBUG(this->get_logger(), "Cached %zu points (from %zu raw)", 
                 cached_points_.size(), cloud_raw->points.size());
  }

  
  void computeAndPublish()
  {
    auto merged_costmap = buildMergedCostmap();
    if (!merged_costmap) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Failed to build merged costmap");
      return;
    }
    
    costmap_converter_->setCostmap2D(merged_costmap.get());
    costmap_converter_->compute();
    
    auto polygons = costmap_converter_->getPolygons();
    if (!polygons || polygons->empty()) {
      RCLCPP_DEBUG(this->get_logger(), "No obstacles detected");
      return;
    }
    
    auto obstacles_msg = std::make_shared<d2_costmap_converter_msgs::msg::ObstacleArrayMsg>();
    obstacles_msg->header.stamp = this->now();
    obstacles_msg->header.frame_id = target_frame_;
    
    for (const auto& poly : *polygons) {
      d2_costmap_converter_msgs::msg::ObstacleMsg obstacle;
      obstacle.polygon = poly;
      obstacles_msg->obstacles.push_back(obstacle);
    }
    
    obstacle_pub_->publish(*obstacles_msg);
    // publishObstaclesAsMarker(obstacles_msg);
    
    RCLCPP_DEBUG(this->get_logger(), "Published %zu obstacles", 
                 obstacles_msg->obstacles.size());
  }

  std::shared_ptr<nav2_costmap_2d::Costmap2D> buildMergedCostmap()
  {
    // ロボット位置取得（mapとして扱う）
    double robot_x = 0.0;
    double robot_y = 0.0;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      robot_x = robot_pose_.position.x;
      robot_y = robot_pose_.position.y;
    }

    nav_msgs::msg::OccupancyGrid::SharedPtr costmap_copy;
    {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      if (!cached_costmap_) return nullptr;
      costmap_copy = cached_costmap_;
    }
    
    double resolution = costmap_copy->info.resolution;
    
    double half_x = submap_size_x_ / 2.0;
    double half_y = submap_size_y_ / 2.0;
    double submap_origin_x = robot_x - half_x;
    double submap_origin_y = robot_y - half_y;
    
    int submap_width = static_cast<int>(std::ceil(submap_size_x_ / resolution));
    int submap_height = static_cast<int>(std::ceil(submap_size_y_ / resolution));
    
    auto merged = std::make_shared<nav2_costmap_2d::Costmap2D>(
      submap_width, submap_height, resolution, 
      submap_origin_x, submap_origin_y);
    
    unsigned char* merged_data = merged->getCharMap();
    std::fill(merged_data, merged_data + submap_width * submap_height, 
              nav2_costmap_2d::FREE_SPACE);
    
    double costmap_origin_x = costmap_copy->info.origin.position.x;
    double costmap_origin_y = costmap_copy->info.origin.position.y;
    
    for (int sy = 0; sy < submap_height; ++sy) {
      for (int sx = 0; sx < submap_width; ++sx) {
        double wx = submap_origin_x + sx * resolution;
        double wy = submap_origin_y + sy * resolution;
        
        int cx = static_cast<int>((wx - costmap_origin_x) / resolution);
        int cy = static_cast<int>((wy - costmap_origin_y) / resolution);
        
        if (cx >= 0 && cx < static_cast<int>(costmap_copy->info.width) &&
            cy >= 0 && cy < static_cast<int>(costmap_copy->info.height)) {
          int costmap_idx = cy * costmap_copy->info.width + cx;
          int8_t cost = costmap_copy->data[costmap_idx];
          
          if (cost == -1) {
            merged_data[sy * submap_width + sx] = nav2_costmap_2d::NO_INFORMATION;
          } else if (cost >= costmap_threshold_) {
            merged_data[sy * submap_width + sx] = nav2_costmap_2d::LETHAL_OBSTACLE;
          }
        }
      }
    }
    
    std::vector<pcl::PointXYZ> points_copy;
    {
      std::lock_guard<std::mutex> lock(pointcloud_mutex_);
      points_copy = cached_points_;
    }
    
    int points_added = 0;
    for (const auto &pt : points_copy) {
      int sx = static_cast<int>((pt.x - submap_origin_x) / resolution);
      int sy = static_cast<int>((pt.y - submap_origin_y) / resolution);
      
      if (sx >= 0 && sx < submap_width && sy >= 0 && sy < submap_height) {
        merged_data[sy * submap_width + sx] = nav2_costmap_2d::LETHAL_OBSTACLE;
        points_added++;
      }
    }
    
    return merged;
  }

  void publishObstaclesAsMarker(
    const d2_costmap_converter_msgs::msg::ObstacleArrayMsg::SharedPtr msg)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = msg->header;
    marker.ns = "obstacles";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.color.r = 1.0;
    marker.color.a = 0.8;
    
    for (const auto& obstacle : msg->obstacles) {
      const auto& points = obstacle.polygon.points;
      if (points.size() < 2) continue;
      
      for (size_t i = 0; i < points.size(); ++i) {
        size_t next_i = (i + 1) % points.size();
        
        geometry_msgs::msg::Point p1, p2;
        p1.x = points[i].x;
        p1.y = points[i].y;
        p1.z = 0.0;
        p2.x = points[next_i].x;
        p2.y = points[next_i].y;
        p2.z = 0.0;
        
        marker.points.push_back(p1);
        marker.points.push_back(p2);
      }
    }
    
    obstacle_marker_pub_->publish(marker);
  }

  pluginlib::ClassLoader<d2_costmap_converter::BaseCostmapToPolygons> costmap_converter_loader_;
  std::shared_ptr<d2_costmap_converter::BaseCostmapToPolygons> costmap_converter_;
  
  rclcpp::Publisher<d2_costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr obstacle_marker_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr compute_timer_;
  
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  
  std::mutex costmap_mutex_;
  std::mutex pointcloud_mutex_;
  std::mutex odom_mutex_;
  nav_msgs::msg::OccupancyGrid::SharedPtr cached_costmap_;
  std::vector<pcl::PointXYZ> cached_points_;
  geometry_msgs::msg::Pose robot_pose_;
  
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;
  
  double converter_rate_;
  int costmap_threshold_;
  double submap_size_x_;
  double submap_size_y_;
  std::string target_frame_;
  double pc_min_dist_;
  double pc_max_dist_;
  double pc_min_height_;
  double pc_max_height_;
};

} // namespace d2_teb_local_planner
