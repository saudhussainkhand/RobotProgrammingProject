#include <chrono>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"  // For resetting covariance

#include "search.hpp"
#include "distance_map.hpp"

using namespace std;
using namespace std::chrono_literals;

class SimplePlanner : public rclcpp::Node {

    private:

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
        geometry_msgs::msg::PoseStamped goal_;

        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_;
        nav_msgs::msg::OccupancyGrid map_;
        cv::Mat distance_map_;
        
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        geometry_msgs::msg::PoseWithCovarianceStamped amcl_pose_;  // Changed to use AMCL pose

        planner::Node root_node;
        planner::Node reached_node_;

        rclcpp::TimerBase::SharedPtr path_timer_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher_;  // For resetting AMCL
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;  // Laser scan subscription
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;  // AMCL pose subscription
        nav_msgs::msg::Path path_;

        // Function to handle receiving the map (occupancy grid)
        void map_received_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            map_ = *msg;
            distance_map_ = planner::compute_distance_map(map_);
            RCLCPP_INFO(this->get_logger(), "Map received: width=%d height=%d", msg->info.width, msg->info.height);
        }

        void goal_received_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            clear_markers();  
            goal_.header = msg->header;
            goal_.pose = msg->pose;
            RCLCPP_INFO(this->get_logger(), "New goal received: '%f %f %f'", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

            // Start path calculation based on current map, robot, and goal positions
            calculate_path();
        }

        void amcl_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            amcl_pose_ = *msg;
            RCLCPP_INFO(this->get_logger(), "AMCL Pose updated: %f, %f", amcl_pose_.pose.pose.position.x, amcl_pose_.pose.pose.position.y);
    }


        void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "Laser scan data received with %zu ranges", msg->ranges.size());
            // Optional: Add obstacle avoidance or visualization logic using the laser scan data.
        }

        void calculate_path() {
            int robot_col = int(amcl_pose_.pose.pose.position.x / map_.info.resolution);
            int robot_row = map_.info.height - int(amcl_pose_.pose.pose.position.y / map_.info.resolution) - 1;
            int robot_vector_index = (map_.info.height - robot_row - 1) * map_.info.width + robot_col;
            int robot_closest_object_distance = distance_map_.at<int>(robot_row, robot_col);

            root_node = planner::Node(robot_row, robot_col, 0, robot_vector_index, robot_closest_object_distance, 0, nullptr);

            int goal_col = int(goal_.pose.position.x / map_.info.resolution);
            int goal_row = map_.info.height - int(goal_.pose.position.y / map_.info.resolution) - 1;
            int goal_vector_index = (map_.info.height - goal_row - 1) * map_.info.width + goal_col;
            int goal_closest_object_distance = distance_map_.at<int>(goal_row, goal_col);

            planner::Node goal_node(goal_row, goal_col, -1, goal_vector_index, goal_closest_object_distance, 0);

            RCLCPP_INFO(this->get_logger(), "Path planning: Start(%d, %d) -> Goal(%d, %d)", root_node.row, root_node.col, goal_node.row, goal_node.col);

            reached_node_ = planner::search(root_node, goal_node, distance_map_);
            if (reached_node_.equals(goal_node)) {
                RCLCPP_INFO(this->get_logger(), "Path successfully calculated.");
                build_path();
            } else {
                RCLCPP_INFO(this->get_logger(), "Goal unreachable.");
            }
        }

        void build_path() {
            path_.header.frame_id = "map";
            path_.poses.clear();
            vector<planner::Node*> path_nodes;
            path_nodes.push_back(&reached_node_);
            planner::Node* current = reached_node_.parent;
            while (current != nullptr) {
                path_nodes.push_back(current);
                current = current->parent;
            }

            for (auto i = path_nodes.rbegin(); i != path_nodes.rend(); ++i) {
                geometry_msgs::msg::PoseStamped pose_msg;
                pose_msg.header.stamp = this->now();
                pose_msg.header.frame_id = "map";  // Use "map" frame to ensure consistency

                auto& node = *i;
                pose_msg.pose.position.x = node->col * map_.info.resolution;
                pose_msg.pose.position.y = (map_.info.height - 1 - node->row) * map_.info.resolution;
                path_.poses.push_back(pose_msg);
            }

            path_publisher_->publish(path_);
        }

        void path_timer_callback() {
            if (path_.poses.empty()) return;

            publish_velocity_to_goal(amcl_pose_, path_.poses[0]);  // Move toward the first pose in the path

            double distance_to_next = calculate_distance(amcl_pose_, path_.poses[0]);
            if (distance_to_next < 0.2) {  // Threshold for reaching a waypoint
                path_.poses.erase(path_.poses.begin());
            }

            path_.header.stamp = this->now();
            path_publisher_->publish(path_);
        }

        double calculate_distance(const geometry_msgs::msg::PoseWithCovarianceStamped& current_pose, const geometry_msgs::msg::PoseStamped& goal_pose) {
            double dx = goal_pose.pose.position.x - current_pose.pose.pose.position.x;
            double dy = goal_pose.pose.position.y - current_pose.pose.pose.position.y;
            return sqrt(dx * dx + dy * dy);
        }

        void publish_velocity_to_goal(const geometry_msgs::msg::PoseWithCovarianceStamped& current_pose, const geometry_msgs::msg::PoseStamped& goal_pose) {
            double k_linear = 1.5;
            double k_angular = 0.3;

            double dx = goal_pose.pose.position.x - current_pose.pose.pose.position.x;
            double dy = goal_pose.pose.position.y - current_pose.pose.pose.position.y;
            double distance_to_goal = sqrt(dx * dx + dy * dy);

            tf2::Quaternion q(
                current_pose.pose.pose.orientation.x,
                current_pose.pose.pose.orientation.y,
                current_pose.pose.pose.orientation.z,
                current_pose.pose.pose.orientation.w
            );
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            double angle_to_goal = atan2(dy, dx) - yaw;

            while (angle_to_goal > M_PI) angle_to_goal -= 2 * M_PI;
            while (angle_to_goal < -M_PI) angle_to_goal += 2 * M_PI;

            geometry_msgs::msg::Twist cmd_vel;
            if (fabs(angle_to_goal) > 0.05) {
                cmd_vel.angular.z = std::clamp(k_angular * angle_to_goal, -0.3, 0.3);
                cmd_vel.linear.x = 0.0;  // Prioritize turning
            } else {
                cmd_vel.linear.x = std::min(k_linear * distance_to_goal, 0.5);
                cmd_vel.angular.z = 0.0;
            }

            cmd_vel_publisher_->publish(cmd_vel);
        }

        void clear_markers() {
            visualization_msgs::msg::MarkerArray marker_array;
            visualization_msgs::msg::Marker marker;

            marker.action = visualization_msgs::msg::Marker::DELETEALL;
            marker.header.frame_id = "map";
            marker.header.stamp = this->now();

            marker_array.markers.push_back(marker);
            marker_publisher_->publish(marker_array);
        }

    public:

        SimplePlanner() : rclcpp::Node("simple_planner") {
            rclcpp::QoS qos(rclcpp::KeepLast(10));

            sub_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("map", qos.transient_local(), std::bind(&SimplePlanner::map_received_callback, this, std::placeholders::_1));
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            sub_goal_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose", qos.durability_volatile(), std::bind(&SimplePlanner::goal_received_callback, this, std::placeholders::_1));
            amcl_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", 10, std::bind(&SimplePlanner::amcl_pose_callback, this, std::placeholders::_1));
            path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("path", qos.transient_local());
            cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
            marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker_array", 10);
            initial_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
            laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/base_scan", 10, std::bind(&SimplePlanner::laser_callback, this, std::placeholders::_1));
            path_timer_ = this->create_wall_timer(200ms, std::bind(&SimplePlanner::path_timer_callback, this));
        }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimplePlanner>());
    rclcpp::shutdown();
    return 0;
}