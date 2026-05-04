// src/lifecycle_manager.cpp
#include "perception_system/lifecycle_manager.hpp"

#include <chrono>
#include <thread>

#include "common/xlogger.hpp"

namespace perception_system {

using namespace std::chrono_literals;
using ChangeState = lifecycle_msgs::srv::ChangeState;
using GetState = lifecycle_msgs::srv::GetState;
using State = lifecycle_msgs::msg::State;
using Transition = lifecycle_msgs::msg::Transition;

LifecycleManager::LifecycleManager(const rclcpp::NodeOptions& options)
    : rclcpp::Node("lifecycle_manager", options) {
  const std::vector<std::string> node_names = {
      "camera_node",
      "image_proc_node",
      "detector_node",
      "visualize_node",
  };
  register_managed_nodes(node_names);

  reset_node_srv_ = create_service<srv::ResetNode>(
      "~/reset_node",
      [this](const std::shared_ptr<srv::ResetNode::Request> req,
             std::shared_ptr<srv::ResetNode::Response> resp) { on_reset_node(req, resp); });

  health_check_timer_ = create_wall_timer(5s, [this]() { on_health_check_timer(); });

  LOG_INFO("manager nodes=%zu", managed_nodes_.size());
}

bool LifecycleManager::startup() {
  if (!configure_all()) return false;
  if (!activate_all()) return false;
  return true;
}

void LifecycleManager::shutdown() {
  is_shutting_down_.store(true, std::memory_order_relaxed);
  health_check_timer_->cancel();
  shutdown_all();
}

bool LifecycleManager::change_state(const std::string& node_name,
                                    uint8_t transition,
                                    std::chrono::seconds timeout) {
  for (auto& node : managed_nodes_) {
    if (node.name == node_name) {
      return transition_node(node, transition, timeout);
    }
  }
  LOG_ERROR("node not found: %s", node_name.c_str());
  return false;
}

void LifecycleManager::on_reset_node(const std::shared_ptr<srv::ResetNode::Request> req,
                                     std::shared_ptr<srv::ResetNode::Response> resp) {
  if (is_shutting_down_.load(std::memory_order_relaxed)) {
    resp->accepted = false;
    return;
  }
  LOG_WARN("[%s] reset requested by HealthMonitor", req->node_name.c_str());

  std::thread([this, node_name = req->node_name]() {
    if (is_shutting_down_.load(std::memory_order_relaxed)) return;
    const bool ok = change_state(node_name, Transition::TRANSITION_DEACTIVATE, 5s)
                    && change_state(node_name, Transition::TRANSITION_CLEANUP, 5s)
                    && change_state(node_name, Transition::TRANSITION_CONFIGURE, 10s)
                    && change_state(node_name, Transition::TRANSITION_ACTIVATE, 10s);
    if (!ok) {
      LOG_ERROR("[%s] reset failed", node_name.c_str());
    } else {
      LOG_WARN("[%s] reset done", node_name.c_str());
    }
  }).detach();

  resp->accepted = true;
}

void LifecycleManager::register_managed_nodes(const std::vector<std::string>& names) {
  for (const auto& name : names) {
    ManagedNode mn;
    mn.name = name;
    mn.change_state_client = create_client<ChangeState>("/" + name + "/change_state");
    mn.get_state_client = create_client<GetState>("/" + name + "/get_state");
    managed_nodes_.push_back(std::move(mn));
  }
}

bool LifecycleManager::configure_all() {
  for (auto& node : managed_nodes_) {
    if (!transition_node(node, Transition::TRANSITION_CONFIGURE, 10s)) {
      LOG_ERROR("configure failed: %s", node.name.c_str());
      return false;
    }
  }
  return true;
}

bool LifecycleManager::activate_all() {
  for (auto& node : managed_nodes_) {
    if (!transition_node(node, Transition::TRANSITION_ACTIVATE, 10s)) {
      LOG_ERROR("activate failed: %s", node.name.c_str());
      return false;
    }
  }
  return true;
}

void LifecycleManager::shutdown_all() {
  for (auto it = managed_nodes_.rbegin(); it != managed_nodes_.rend(); ++it) {
    auto req = std::make_shared<GetState::Request>();
    if (!it->get_state_client->wait_for_service(2s)) {
      LOG_WARN("[%s] get_state unavailable during shutdown", it->name.c_str());
      continue;
    }

    auto future = it->get_state_client->async_send_request(req);
    if (future.wait_for(1s) != std::future_status::ready) {
      LOG_WARN("[%s] get_state timeout during shutdown", it->name.c_str());
      continue;
    }

    uint8_t state = future.get()->current_state.id;

    if (state == State::PRIMARY_STATE_ACTIVE) {
      if (!transition_node(*it, Transition::TRANSITION_DEACTIVATE, 5s)) {
        LOG_WARN("deactivate failed: %s", it->name.c_str());
      }
      state = State::PRIMARY_STATE_INACTIVE;
    }

    if (state == State::PRIMARY_STATE_INACTIVE) {
      if (!transition_node(*it, Transition::TRANSITION_CLEANUP, 5s)) {
        LOG_WARN("cleanup failed: %s", it->name.c_str());
      }
    }
  }
}

bool LifecycleManager::transition_node(ManagedNode& node,
                                       uint8_t transition,
                                       std::chrono::seconds timeout) {
  if (!node.change_state_client->wait_for_service(timeout)) {
    LOG_WARN("[%s] change_state unavailable", node.name.c_str());
    return false;
  }

  auto req = std::make_shared<ChangeState::Request>();
  req->transition.id = transition;

  auto future = node.change_state_client->async_send_request(req);

  if (future.wait_for(timeout) != std::future_status::ready) {
    LOG_ERROR("[%s] transition %u timed out", node.name.c_str(), transition);
    return false;
  }

  const bool ok = future.get()->success;
  if (!ok) {
    LOG_ERROR("[%s] transition %u rejected", node.name.c_str(), transition);
  }
  return ok;
}

void LifecycleManager::on_health_check_timer() {
  if (managed_nodes_.empty()) return;
  if (is_shutting_down_.load(std::memory_order_relaxed)) return;

  auto& node = managed_nodes_[health_check_index_];
  health_check_index_ = (health_check_index_ + 1) % managed_nodes_.size();
  check_node_state_async(node);
}

void LifecycleManager::check_node_state_async(ManagedNode& node) {
  if (!node.get_state_client->service_is_ready()) {
    LOG_WARN("[%s] get_state not ready", node.name.c_str());
    return;
  }

  auto req = std::make_shared<GetState::Request>();

  node.get_state_client->async_send_request(
      req,
      [this, name = node.name](rclcpp::Client<GetState>::SharedFuture future) {
        if (is_shutting_down_.load(std::memory_order_relaxed)) return;

        const uint8_t state = future.get()->current_state.id;
        if (state == State::PRIMARY_STATE_ACTIVE) return;

        LOG_WARN("[%s] state=%u, trying activate", name.c_str(), state);

        if (state == State::PRIMARY_STATE_INACTIVE) {
          for (auto& n : managed_nodes_) {
            if (n.name != name) continue;
            if (!n.change_state_client->service_is_ready()) break;
            auto req2 = std::make_shared<ChangeState::Request>();
            req2->transition.id = Transition::TRANSITION_ACTIVATE;
            n.change_state_client->async_send_request(
                req2,
                [name](rclcpp::Client<ChangeState>::SharedFuture f) {
                  if (!f.get()->success) {
                    LOG_ERROR("[%s] activate failed", name.c_str());
                  }
                });
            break;
          }
        } else if (state == State::PRIMARY_STATE_FINALIZED) {
          LOG_ERROR("[%s] finalized", name.c_str());
        }
      });
}

}  // namespace perception_system
