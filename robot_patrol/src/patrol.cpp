#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/twist.hpp>

class Patrol : public rclcpp::Node {
public:
  Patrol(const std::string &node_name = "patrol_node")
      : Node(node_name), node_name_(node_name) {

    auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);

    subscriber_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/fastbot_1/scan", qos,
        std::bind(&Patrol::laserscan_callback, this, std::placeholders::_1));

    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/fastbot_1/cmd_vel", 10);

    control_timer_ =
        this->create_wall_timer(std::chrono::milliseconds(100),
                                std::bind(&Patrol::control_callback, this));

    RCLCPP_INFO(this->get_logger(), "%s ready...", node_name_.c_str());
  }

  // public variables

  // public functions

private:
  std::string node_name_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      subscriber_laser_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

  // private variables
  const double distance_thres = 0.35;
  bool obstacle_found;
  double turn_dir;

  // subscriptions callback functions
  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    // Define the sectors with their index ranges
    // Scan array has 200 values for 2pi range
    // First value is front, moving counter-clockwise
    // 50th value is left, 150th is right
    // Front 180° corresponds to indexes [0 - 49] and [150 - 199]
    // Using a range of -20°, +20° as front == indexes [0:10],[189:199]
    std::map<std::string, std::pair<int, int>> sectors = {
        {"Front_Left", {0, 10}},
        {"Left", {11, 49}},
        {"Right", {150, 188}},
        {"Front_Right", {150, 199}}};

    // Initialize the minimum distances for each sector
    std::map<std::string, float> min_distances, max_distances;
    for (const auto &sector : sectors) {
      min_distances[sector.first] = std::numeric_limits<float>::infinity();
      max_distances[sector.first] = std::numeric_limits<float>::infinity();
    }

    // Find the minimum distance in each sector
    for (const auto &sector : sectors) {
      int start_idx = sector.second.first;
      int end_idx = sector.second.second;

      // Ensure the index range is within bounds
      if (start_idx < static_cast<int>(msg->ranges.size()) &&
          end_idx < static_cast<int>(msg->ranges.size())) {
        auto start_it = msg->ranges.begin() + start_idx;
        auto end_it = msg->ranges.begin() + end_idx + 1;

        if (start_it < end_it) {
          min_distances[sector.first] = *std::min_element(start_it, end_it);
          max_distances[sector.first] = *std::max_element(start_it, end_it);
        }
      }
    }

    // Define the threshold for obstacle detection
    float obstacle_threshold = 0.35f; // meters

    // Determine detected obstacles
    std::map<std::string, bool> detections;
    for (const auto &distance : min_distances) {
      detections[distance.first] = distance.second < obstacle_threshold;
    }

    if (detections["Front_Left"] || detections["Front_Right"]) {
      RCLCPP_INFO(this->get_logger(), "Obstacle ahead.");
      obstacle_found = true;

      // Determine the safest direction to turn
      if ((std::max(max_distances["Front_Right"], max_distances["Right"]) <
           std::max(max_distances["Left"], max_distances["Front_Left"])) &&
          std::max(max_distances["Left"], max_distances["Front_Left"]) >
              distance_thres) {
        turn_dir = 0.5;
        RCLCPP_INFO(this->get_logger(), "Safest direction is left.");
      } else if (std::max(max_distances["Right"],
                          max_distances["Front_Right"]) > distance_thres) {
        turn_dir = -0.5;
        RCLCPP_INFO(this->get_logger(), "Safest direction is right.");
      } else {
        turn_dir = 0;
        RCLCPP_INFO(this->get_logger(), "No safe direction.");
      }
    } else {
      obstacle_found = false;
    }
  }

  void control_callback() {

    auto vel_msg = geometry_msgs::msg::Twist();

    if (!obstacle_found) {
      vel_msg.linear.x = 0.1;
      vel_msg.angular.z = 0.0;
    } else if (turn_dir != 0) {
      vel_msg.linear.x = 0.05;
      vel_msg.angular.z = turn_dir;
    } else {
      // No safe diretion case
      vel_msg.linear.x = 0.0;
      vel_msg.angular.z = 0.0;
    }
    cmd_vel_publisher_->publish(vel_msg);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Patrol>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
