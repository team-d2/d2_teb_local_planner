#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <d2_costmap_converter_msgs/msg/obstacle_array_msg.hpp>
#include <d2_costmap_converter/costmap_converter_interface.h>
#include <pluginlib/class_loader.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace d2_teb_local_planner
{

class ObstacleFusionComponent : public rclcpp::Node
{
public:
  explicit ObstacleFusionComponent(const rclcpp::NodeOptions & options)
  : Node("obstacle_fusion_component", options),
    costmap_converter_loader_("d2_costmap_converter", 
                              "d2_costmap_converter::BaseCostmapToPolygons")
  {
    // Parameters
    this->declare_parameter("costmap_converter_plugin", 
                           "d2_costmap_converter::CostmapToPolygonsDBSMCCH");
    this->declare_parameter("costmap_converter_rate", 5);
    this->declare_parameter("point_obstacle_radius", 0.1);
    this->declare_parameter("costmap_lethal_threshold", 250);
    
    std::string plugin_name;
    this->get_parameter("costmap_converter_plugin", plugin_name);
    this->get_parameter("costmap_converter_rate", converter_rate_);
    this->get_parameter("point_obstacle_radius", point_radius_);
    this->get_parameter("costmap_lethal_threshold", costmap_threshold_);

    // Load costmap converter plugin
    try {
      costmap_converter_ = costmap_converter_loader_.createSharedInstance(plugin_name);
      auto converter_node = std::make_shared<rclcpp::Node>("costmap_converter_node");
      costmap_converter_->initialize(converter_node);
      RCLCPP_INFO(this->get_logger(), "Loaded costmap converter: %s", plugin_name.c_str());
    } catch (const pluginlib::PluginlibException& ex) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load plugin: %s", ex.what());
      costmap_converter_.reset();
    }

    // Publishers
    obstacle_pub_ = this->create_publisher<d2_costmap_converter_msgs::msg::ObstacleArrayMsg>(
        "/obstacles", rclcpp::SensorDataQoS());

    // Subscribers
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap", rclcpp::QoS(1).transient_local().reliable(),
        std::bind(&ObstacleFusionComponent::costmapCallback, this, std::placeholders::_1));
    
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/points", rclcpp::SensorDataQoS(),
        std::bind(&ObstacleFusionComponent::pointcloudCallback, this, std::placeholders::_1));

    // Timer for publishing
    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / converter_rate_),
        std::bind(&ObstacleFusionComponent::publishObstacles, this));

    RCLCPP_INFO(this->get_logger(), "ObstacleFusionComponent initialized");
  }

private:
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    last_costmap_ = msg;
    
    if (costmap_converter_) {
      // Convert OccupancyGrid to Costmap2D
      updateCostmapConverter(msg);
    }
  }

  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(pointcloud_mutex_);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    
    point_obstacles_.clear();
    for (const auto& pt : cloud->points) {
      if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
        // Z軸でフィルタリング（例: 0.1m～2.0mの範囲）
        if (pt.z > 0.1 && pt.z < 2.0) {
          point_obstacles_.push_back(pt);
        }
      }
    }
  }

  void updateCostmapConverter(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    // Create temporary Costmap2D from OccupancyGrid
    auto costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(
        msg->info.width, msg->info.height, msg->info.resolution,
        msg->info.origin.position.x, msg->info.origin.position.y);
    
    unsigned char* data = costmap->getCharMap();
    for (size_t i = 0; i < msg->data.size(); ++i) {
      if (msg->data[i] == -1) {
        data[i] = nav2_costmap_2d::NO_INFORMATION;
      } else if (msg->data[i] >= costmap_threshold_) {
        data[i] = nav2_costmap_2d::LETHAL_OBSTACLE;
      } else {
        data[i] = nav2_costmap_2d::FREE_SPACE;
      }
    }
    
    costmap_converter_->setCostmap2D(costmap.get());
    costmap_converter_->compute();
  }

  void publishObstacles()
  {
    auto obstacles_msg = std::make_shared<d2_costmap_converter_msgs::msg::ObstacleArrayMsg>();
    obstacles_msg->header.stamp = this->now();
    obstacles_msg->header.frame_id = "odom";

    // Add costmap-based obstacles
    {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      if (costmap_converter_) {
        auto polygons = costmap_converter_->getPolygons();
        if (polygons) {
          for (const auto& poly : *polygons) {
            d2_costmap_converter_msgs::msg::ObstacleMsg obstacle;
            obstacle.polygon = poly;
            obstacles_msg->obstacles.push_back(obstacle);
          }
        }
      }
    }

    // Add point cloud obstacles
    {
      std::lock_guard<std::mutex> lock(pointcloud_mutex_);
      for (const auto& pt : point_obstacles_) {
        d2_costmap_converter_msgs::msg::ObstacleMsg obstacle;
        geometry_msgs::msg::Point32 point;
        point.x = pt.x;
        point.y = pt.y;
        point.z = 0.0;
        obstacle.polygon.points.push_back(point);
        obstacle.radius = point_radius_;
        obstacle.id = -1;
        obstacles_msg->obstacles.push_back(obstacle);
      }
    }

    if (!obstacles_msg->obstacles.empty()) {
      obstacle_pub_->publish(*obstacles_msg);
      RCLCPP_DEBUG(this->get_logger(), "Published %zu obstacles", 
                   obstacles_msg->obstacles.size());
    }
  }

  pluginlib::ClassLoader<d2_costmap_converter::BaseCostmapToPolygons> costmap_converter_loader_;
  std::shared_ptr<d2_costmap_converter::BaseCostmapToPolygons> costmap_converter_;
  
  rclcpp::Publisher<d2_costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacle_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  
  std::mutex costmap_mutex_;
  std::mutex pointcloud_mutex_;
  nav_msgs::msg::OccupancyGrid::SharedPtr last_costmap_;
  std::vector<pcl::PointXYZ> point_obstacles_;
  
  int converter_rate_;
  double point_radius_;
  int costmap_threshold_;
};

} // namespace d2_teb_local_planner
