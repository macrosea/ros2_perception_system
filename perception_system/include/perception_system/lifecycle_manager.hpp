#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "perception_system/srv/reset_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace perception_system {

struct ManagedNode {
  std::string name;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_client;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr get_state_client;
};

class LifecycleManager : public rclcpp::Node {
 public:
  explicit LifecycleManager(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  bool startup();
  void shutdown();

  bool change_state(const std::string& node_name,
                    uint8_t transition,
                    std::chrono::seconds timeout = std::chrono::seconds(5));

 private:
  void register_managed_nodes(const std::vector<std::string>& names);
  bool configure_all();
  bool activate_all();
  void shutdown_all();
  bool transition_node(ManagedNode& node, uint8_t transition, std::chrono::seconds timeout);

  void on_health_check_timer();
  void check_node_state_async(ManagedNode& node);

  void on_reset_node(const std::shared_ptr<srv::ResetNode::Request> req,
                     std::shared_ptr<srv::ResetNode::Response> resp);

  std::vector<ManagedNode> managed_nodes_;
  rclcpp::TimerBase::SharedPtr health_check_timer_;
  rclcpp::Service<srv::ResetNode>::SharedPtr reset_node_srv_;
  size_t health_check_index_{0};
  std::atomic<bool> is_shutting_down_{false};
};

}  // namespace perception_system
