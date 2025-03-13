#include <algorithm>
#include <deque>
#include <utility>

#include "auto_apms_behavior_tree/node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#define INPUT_KEY_FOV "cone_width"
#define INPUT_KEY_DIST "distance"
#define INPUT_KEY_THRESH "threshold"

// Applies a median filter to the distance measurements at a specific index
// across a sliding window of LaserScan messages
std::vector<float> apply_median_filter(const std::deque<sensor_msgs::msg::LaserScan> & scans, int index)
{
  std::vector<float> values;

  // Collect valid measurements (non-zero and within valid range)
  for (const auto & scan : scans) {
    if (scan.ranges[index] > scan.range_min && scan.ranges[index] < scan.range_max && scan.ranges[index] != 0.0) {
      values.push_back(scan.ranges[index]);
    }
  }

  // If no valid measurements, return infinity
  if (values.empty()) {
    return {std::numeric_limits<float>::infinity()};
  }

  // Sort and return the median value
  std::sort(values.begin(), values.end());
  return {values[values.size() / 2]};
}

namespace uas_ros2::bt
{

class DetectObstacle : public auto_apms_behavior_tree::core::RosSubscriberNode<sensor_msgs::msg::LaserScan>
{
public:
  explicit DetectObstacle(const std::string & instance_name, const Config & config, Context context)
  : RosSubscriberNode(instance_name, config, context, rclcpp::SensorDataQoS()) {};

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {BT::InputPort<double>(
         INPUT_KEY_FOV, 180,
         "Width [°] of the forward facing cone (centered around 0°) specifying the field of view to be considered."),
       BT::InputPort<double>(INPUT_KEY_DIST, "Distance [m] at which an obstacle will be considered detected."),
       BT::InputPort<double>(
         INPUT_KEY_THRESH, 0.5,
         "Threshold [%] determining how many of the measured points must be within the configured range for the "
         "detection to be triggered.")});
  }

  BT::NodeStatus onMessageReceived(const MessageType & msg) override final
  {
    double cone_width_deg;
    if (const BT::Expected<double> expected = getInput<double>(INPUT_KEY_FOV)) {
      cone_width_deg = expected.value();
      if (cone_width_deg < 0 || cone_width_deg > 360) {
        throw auto_apms_behavior_tree::exceptions::RosNodeError(
          context_.getFullyQualifiedTreeNodeName(this) + " - Invalid cone width. Must be within [0, 360] got " +
          std::to_string(cone_width_deg));
      }
    } else {
      throw auto_apms_behavior_tree::exceptions::RosNodeError(
        context_.getFullyQualifiedTreeNodeName(this) + " - " + expected.error());
    }
    double distance_m;
    if (const BT::Expected<double> expected = getInput<double>(INPUT_KEY_DIST)) {
      distance_m = expected.value();
      if (distance_m < msg.range_min || distance_m > msg.range_max) {
        throw auto_apms_behavior_tree::exceptions::RosNodeError(
          context_.getFullyQualifiedTreeNodeName(this) + " - Invalid distance: Must be within [" +
          std::to_string(msg.range_min) + ", " + std::to_string(msg.range_max) + "] got " + std::to_string(distance_m));
      }
    } else {
      throw auto_apms_behavior_tree::exceptions::RosNodeError(
        context_.getFullyQualifiedTreeNodeName(this) + " - " + expected.error());
    }
    double thresh;
    if (const BT::Expected<double> expected = getInput<double>(INPUT_KEY_THRESH)) {
      thresh = expected.value();
      if (thresh < 0 || thresh > 1) {
        throw auto_apms_behavior_tree::exceptions::RosNodeError(
          context_.getFullyQualifiedTreeNodeName(this) + " - Invalid distance: Must be within [0, 1] got " +
          std::to_string(thresh));
      }
    } else {
      throw auto_apms_behavior_tree::exceptions::RosNodeError(
        context_.getFullyQualifiedTreeNodeName(this) + " - " + expected.error());
    }

    const size_t MAX_WINDOW_SIZE = 5;

    // Add scan message to queue
    scans.push_back(msg);
    if (scans.size() > MAX_WINDOW_SIZE) scans.pop_front();

    // Calculate the indices for the forward-looking cone (centered around 0°)
    int total_points = scans.front().ranges.size();
    int half_cone_points = static_cast<int>((cone_width_deg / 360.0) * total_points / 2);
    int center_index = 0;  // 0° index
    int start_index = std::max(0, center_index - half_cone_points);
    int end_index = std::min(total_points - 1, center_index + half_cone_points);

    int total_points_in_fov = end_index - start_index + 1;
    int obstacle_count = 0;

    // Iterate over the specified field of view and check for obstacles
    for (int i = start_index; i <= end_index; ++i) {
      float filtered_value = apply_median_filter(scans, i)[0];
      if (filtered_value < distance_m && filtered_value != std::numeric_limits<float>::infinity()) {
        obstacle_count++;
      }
    }

    // Calculate the percentage of points detecting an obstacle
    float detected_percentage = static_cast<float>(obstacle_count) / total_points_in_fov;
    return detected_percentage >= thresh ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  std::deque<sensor_msgs::msg::LaserScan> scans;
};

}  // namespace uas_ros2::bt

// Make the nodes discoverable for the class loader
AUTO_APMS_BEHAVIOR_TREE_DECLARE_NODE(uas_ros2::bt::DetectObstacle)