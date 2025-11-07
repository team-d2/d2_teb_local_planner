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
#include <pcl/segmentation/extract_clusters.h>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <chrono>

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
    this->declare_parameter("cluster_tolerance", 0.4);
    this->declare_parameter("cluster_min_points", 6);
    this->declare_parameter("cluster_max_points", 200);
    this->declare_parameter("cluster_max_clusters", 120);
    this->declare_parameter("cluster_max_radius", 0.8);
    this->declare_parameter("submap_front", 10.0);
    this->declare_parameter("submap_rear", 3.0);
    this->declare_parameter("submap_side", 3.0);
    
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
    submap_front_ = this->get_parameter("submap_front").as_double();
    submap_rear_ = this->get_parameter("submap_rear").as_double();
    submap_side_ = this->get_parameter("submap_side").as_double();

    double voxel_size = this->get_parameter("voxel_leaf_size").as_double();
    voxel_filter_.setLeafSize(voxel_size, voxel_size, voxel_size);

    cluster_tolerance_ = this->get_parameter("cluster_tolerance").as_double();
    cluster_min_points_ = std::max<int>(1, this->get_parameter("cluster_min_points").as_int());
    cluster_max_points_ = std::max<int>(cluster_min_points_, this->get_parameter("cluster_max_points").as_int());
    cluster_max_clusters_ = std::max<int>(1, this->get_parameter("cluster_max_clusters").as_int());
    cluster_max_radius_ = this->get_parameter("cluster_max_radius").as_double();

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
        "/obstacles", rclcpp::SensorDataQoS());
    obstacle_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/obstacle_markers", rclcpp::SensorDataQoS());

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
    const std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
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
      RCLCPP_WARN(this->get_logger(), "No obstacles detected");
      return;
    }
    
    auto obstacles_msg = std::make_shared<d2_costmap_converter_msgs::msg::ObstacleArrayMsg>();
    obstacles_msg->header.stamp = this->now();
    obstacles_msg->header.frame_id = target_frame_;
    
    if (polygons) {
      for (const auto& poly : *polygons) {
        d2_costmap_converter_msgs::msg::ObstacleMsg obstacle;
        obstacle.orientation.w = 1.0;
        obstacle.polygon = poly;
        obstacles_msg->obstacles.push_back(obstacle);
      }
    }
    
    appendPointClusters(*obstacles_msg);
    if (obstacles_msg->obstacles.empty()) {
      RCLCPP_WARN(this->get_logger(), "No obstacles to publish after clustering");
      return;
    }
    
    obstacle_pub_->publish(*obstacles_msg);
    publishObstaclesAsMarker(obstacles_msg);

    RCLCPP_DEBUG(this->get_logger(), "Published %zu obstacles",
                 obstacles_msg->obstacles.size());
    const std::chrono::steady_clock::time_point total_end_time = std::chrono::steady_clock::now();
    const auto total_duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(total_end_time - start_time).count();
    RCLCPP_DEBUG(this->get_logger(), "Total compute and publish time: %ld ms", total_duration_ms);
  }

  std::shared_ptr<nav2_costmap_2d::Costmap2D> buildMergedCostmap()
  {
    double robot_x = 0.0;
    double robot_y = 0.0;
    geometry_msgs::msg::Quaternion robot_q;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      robot_x = robot_pose_.position.x;
      robot_y = robot_pose_.position.y;
      robot_q = robot_pose_.orientation;
    }

    nav_msgs::msg::OccupancyGrid::SharedPtr costmap_copy;
    {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      if (!cached_costmap_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No costmap received yet.");
        return nullptr;
      }
      costmap_copy = cached_costmap_;
    }

    double resolution = costmap_copy->info.resolution;
    if (resolution <= 0.0) {
      RCLCPP_ERROR(this->get_logger(), "Invalid costmap resolution: %f", resolution);
      return nullptr;
    }

    // ロボットの向き（ヨー角）を計算
    double qx = robot_q.x;
    double qy = robot_q.y;
    double qz = robot_q.z;
    double qw = robot_q.w;
    double yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    double cos_yaw = std::cos(yaw);
    double sin_yaw = std::sin(yaw);

    // ロボット座標系でのサブマップの4隅の点を定義
    // (前方: +x, 後方: -x, 左: +y, 右: -y)
    std::vector<std::pair<double, double>> local_corners = {
      {submap_front_,  submap_side_},  // 前方左
      {submap_front_, -submap_side_},  // 前方右
      {-submap_rear_, -submap_side_},  // 後方右
      {-submap_rear_,  submap_side_}   // 後方左
    };

    // 4隅の点をmap座標系に変換
    double min_wx = std::numeric_limits<double>::max();
    double max_wx = std::numeric_limits<double>::lowest();
    double min_wy = std::numeric_limits<double>::max();
    double max_wy = std::numeric_limits<double>::lowest();

    for (const auto& local_pt : local_corners) {
      double wx = robot_x + (local_pt.first * cos_yaw - local_pt.second * sin_yaw);
      double wy = robot_y + (local_pt.first * sin_yaw + local_pt.second * cos_yaw);
      min_wx = std::min(min_wx, wx);
      max_wx = std::max(max_wx, wx);
      min_wy = std::min(min_wy, wy);
      max_wy = std::max(max_wy, wy);
    }

    // map座標系で軸並行なバウンディングボックス（サブマップ）を定義
    double submap_origin_x = min_wx;
    double submap_origin_y = min_wy;
    int submap_width = static_cast<int>(std::ceil((max_wx - min_wx) / resolution));
    int submap_height = static_cast<int>(std::ceil((max_wy - min_wy) / resolution));

    auto merged = std::make_shared<nav2_costmap_2d::Costmap2D>(
      submap_width, submap_height, resolution,
      submap_origin_x, submap_origin_y);

    unsigned char* merged_data = merged->getCharMap();
    std::fill(merged_data, merged_data + submap_width * submap_height, nav2_costmap_2d::FREE_SPACE);

    // 元のコストマップからサブマップへデータをコピー
    double costmap_origin_x = costmap_copy->info.origin.position.x;
    double costmap_origin_y = costmap_copy->info.origin.position.y;
    int costmap_w = static_cast<int>(costmap_copy->info.width);
    int costmap_h = static_cast<int>(costmap_copy->info.height);

    for (int sy = 0; sy < submap_height; ++sy) {
      for (int sx = 0; sx < submap_width; ++sx) {
        double wx = submap_origin_x + (sx + 0.5) * resolution;
        double wy = submap_origin_y + (sy + 0.5) * resolution;

        int cx, cy;
        merged->worldToMapNoBounds(wx, wy, cx, cy); // この関数は内部でチェックするため安全

        int costmap_mx = static_cast<int>((wx - costmap_origin_x) / resolution);
        int costmap_my = static_cast<int>((wy - costmap_origin_y) / resolution);

        if (costmap_mx >= 0 && costmap_mx < costmap_w && costmap_my >= 0 && costmap_my < costmap_h) {
          int costmap_idx = costmap_my * costmap_w + costmap_mx;
          int8_t cost = costmap_copy->data[costmap_idx];

          if (cost == -1) {
            merged->setCost(cx, cy, nav2_costmap_2d::NO_INFORMATION);
          } else if (cost >= costmap_threshold_) {
            merged->setCost(cx, cy, nav2_costmap_2d::LETHAL_OBSTACLE);
          }
        }
      }
    }
    return merged;
  }

  void appendPointClusters(d2_costmap_converter_msgs::msg::ObstacleArrayMsg & msg)
  {
    std::vector<pcl::PointXYZ> points_copy;
    {
      std::lock_guard<std::mutex> lock(pointcloud_mutex_);
      points_copy = cached_points_;
    }
    if (points_copy.size() < static_cast<std::size_t>(cluster_min_points_)) {
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->points.resize(points_copy.size());
    for (std::size_t i = 0; i < points_copy.size(); ++i) {
      cloud->points[i] = points_copy[i];
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> extractor;
    extractor.setClusterTolerance(cluster_tolerance_);
    extractor.setMinClusterSize(cluster_min_points_);
    extractor.setMaxClusterSize(cluster_max_points_);
    extractor.setSearchMethod(tree);
    extractor.setInputCloud(cloud);
    extractor.extract(cluster_indices);

    std::size_t appended = 0;
    for (const auto & cluster : cluster_indices) {
      if (appended >= static_cast<std::size_t>(cluster_max_clusters_)) {
        break;
      }
      if (cluster.indices.empty()) {
        continue;
      }

      Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
      for (int idx : cluster.indices) {
        const auto & pt = cloud->points[static_cast<std::size_t>(idx)];
        centroid += Eigen::Vector3d(pt.x, pt.y, pt.z);
      }
      centroid /= static_cast<double>(cluster.indices.size());

      double radius = 0.0;
      for (int idx : cluster.indices) {
        const auto & pt = cloud->points[static_cast<std::size_t>(idx)];
        Eigen::Vector2d diff(pt.x - centroid.x(), pt.y - centroid.y());
        radius = std::max(radius, diff.norm());
      }
      if (cluster_max_radius_ > 0.0) {
        radius = std::min(radius, cluster_max_radius_);
      }

      d2_costmap_converter_msgs::msg::ObstacleMsg obstacle;
      obstacle.id = static_cast<int32_t>(msg.obstacles.size());
      obstacle.orientation.w = 1.0;
      obstacle.radius = radius;
      obstacle.polygon.points.resize(1);
      obstacle.polygon.points[0].x = static_cast<float>(centroid.x());
      obstacle.polygon.points[0].y = static_cast<float>(centroid.y());
      obstacle.polygon.points[0].z = static_cast<float>(centroid.z());

      msg.obstacles.push_back(obstacle);
      ++appended;
    }

    RCLCPP_DEBUG(this->get_logger(),
                 "Clustered %zu obstacles from %zu points (kept %zu)",
                 appended, points_copy.size(), msg.obstacles.size());
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

    const int circle_segments = 24;
    const float cross_size = 0.15f;

    for (const auto& obstacle : msg->obstacles) {
      const auto& pts = obstacle.polygon.points;

      if (pts.size() >= 2) {
        for (size_t i = 0; i < pts.size(); ++i) {
          size_t next_i = (i + 1) % pts.size();
          geometry_msgs::msg::Point p1, p2;
          p1.x = pts[i].x; p1.y = pts[i].y; p1.z = pts[i].z;
          p2.x = pts[next_i].x; p2.y = pts[next_i].y; p2.z = pts[next_i].z;
          marker.points.push_back(p1);
          marker.points.push_back(p2);
        }
      } else if (pts.size() == 1 && obstacle.radius > 0.0f) {
        // 円（中心 pts[0]、半径 obstacle.radius）を近似して描画
        geometry_msgs::msg::Point center;
        center.x = pts[0].x; center.y = pts[0].y; center.z = pts[0].z;
        std::vector<geometry_msgs::msg::Point> circ_pts(circle_segments);
        for (int i = 0; i < circle_segments; ++i) {
          double ang = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(circle_segments);
          geometry_msgs::msg::Point p;
          p.x = center.x + static_cast<double>(obstacle.radius) * std::cos(ang);
          p.y = center.y + static_cast<double>(obstacle.radius) * std::sin(ang);
          p.z = center.z;
          circ_pts[i] = p;
        }
        for (int i = 0; i < circle_segments; ++i) {
          int ni = (i + 1) % circle_segments;
          marker.points.push_back(circ_pts[i]);
          marker.points.push_back(circ_pts[ni]);
        }
        // optionally draw center as small cross
        geometry_msgs::msg::Point c1, c2, c3, c4;
        c1.x = center.x - cross_size; c1.y = center.y; c1.z = center.z;
        c2.x = center.x + cross_size; c2.y = center.y; c2.z = center.z;
        c3.x = center.x; c3.y = center.y - cross_size; c3.z = center.z;
        c4.x = center.x; c4.y = center.y + cross_size; c4.z = center.z;
        marker.points.push_back(c1); marker.points.push_back(c2);
        marker.points.push_back(c3); marker.points.push_back(c4);

      } else if (pts.size() == 1) {
        // 半径が無い単一点は小さな十字で可視化
        geometry_msgs::msg::Point center;
        center.x = pts[0].x; center.y = pts[0].y; center.z = pts[0].z;
        geometry_msgs::msg::Point c1, c2, c3, c4;
        c1.x = center.x - cross_size; c1.y = center.y; c1.z = center.z;
        c2.x = center.x + cross_size; c2.y = center.y; c2.z = center.z;
        c3.x = center.x; c3.y = center.y - cross_size; c3.z = center.z;
        c4.x = center.x; c4.y = center.y + cross_size; c4.z = center.z;
        marker.points.push_back(c1); marker.points.push_back(c2);
        marker.points.push_back(c3); marker.points.push_back(c4);
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
  double submap_front_;
  double submap_rear_;
  double submap_side_;
  std::string target_frame_;
  double pc_min_dist_;
  double pc_max_dist_;
  double pc_min_height_;
  double pc_max_height_;

  double cluster_tolerance_;
  int cluster_min_points_;
  int cluster_max_points_;
  int cluster_max_clusters_;
  double cluster_max_radius_;
};

} // namespace d2_teb_local_planner
