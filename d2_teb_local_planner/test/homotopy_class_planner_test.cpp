#include "d2_teb_local_planner/homotopy_class_planner.h"
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

class HomotopyClassPlannerTest : public d2_teb_local_planner::HomotopyClassPlanner {
    public:
    void SetUp(rclcpp::Node::SharedPtr node) {
        d2_teb_local_planner::RobotFootprintModelPtr robot_model;
        d2_teb_local_planner::TebVisualizationPtr visualization;
        d2_teb_local_planner::HomotopyClassPlannerPtr homotopy_class_planner;

        robot_model.reset(new d2_teb_local_planner::CircularRobotFootprint(0.25));

        obstacles.push_back(
            d2_teb_local_planner::ObstaclePtr(new d2_teb_local_planner::PointObstacle(2, 2))
        );

        obstacles.push_back(
            d2_teb_local_planner::ObstaclePtr(new d2_teb_local_planner::PointObstacle(2, 3))
        );

        obstacles.push_back(
            d2_teb_local_planner::ObstaclePtr(new d2_teb_local_planner::PointObstacle(2, 4))
        );

        visualization.reset(
            new d2_teb_local_planner::TebVisualization(node, cfg)
        );

        cfg.hcp.visualize_hc_graph = true;

        initialize(node, cfg, &obstacles, robot_model, visualization);
    }
    d2_teb_local_planner::ObstContainer obstacles;
    d2_teb_local_planner::TebConfig cfg;
};

TEST(test, test) {
    HomotopyClassPlannerTest test;
    rclcpp::Node::SharedPtr node( new rclcpp::Node("test") );
    test.SetUp(node);

    using namespace d2_teb_local_planner;
    PoseSE2 start(0, 0, 0);
    PoseSE2 goal(5, 5, 0);
    geometry_msgs::msg::Twist twist;
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.angular.z = 0;

    std::vector<geometry_msgs::msg::PoseStamped> initial_plan;
    
    geometry_msgs::msg::PoseStamped start_pose;
    geometry_msgs::msg::PoseStamped goal_pose;

    start.toPoseMsg(start_pose.pose);
    goal.toPoseMsg(goal_pose.pose);
    
    initial_plan.push_back(start_pose);
    initial_plan.push_back(goal_pose);

    test.plan(initial_plan, &twist, false);
    //homotopy_class_planner_->exploreEquivalenceClassesAndInitTebs(start, goal, 0.3, &twist);
    
    rclcpp::Rate rate(10);
    while(rclcpp::ok()) {
        test.visualize();
        rclcpp::spin_some(node);
        rate.sleep();
    }

    ASSERT_TRUE(true);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}