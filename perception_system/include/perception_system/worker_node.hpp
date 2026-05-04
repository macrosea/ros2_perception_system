#pragma once

#include <atomic>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <thread>

#include "common/affinity.hpp"
#include "common/latest_slot.hpp"
#include "common/xlogger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace perception_system {

namespace {
constexpr int kWorkerJoinTimeoutSec = 5;
}

template <typename MsgT>
class WorkerNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit WorkerNode(const std::string& node_name, const rclcpp::NodeOptions& options)
      : rclcpp_lifecycle::LifecycleNode(node_name, options) {}

  virtual ~WorkerNode() = default;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) final {
    auto ret = OnActivating(state);
    if (ret != CallbackReturn::SUCCESS) {
      return ret;
    }

    LifecycleNode::on_activate(state);
    StartWorker();
    OnActivated(state);

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) final {
    OnDeactivating(state);
    StopWorker();
    LifecycleNode::on_deactivate(state);
    OnDeactivated(state);

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_error(const rclcpp_lifecycle::State& state) final {
    OnErroring(state);
    StopWorker();
    OnErrored(state);

    return CallbackReturn::SUCCESS;
  }

 protected:
  virtual CallbackReturn OnActivating(const rclcpp_lifecycle::State&) {
    return CallbackReturn::SUCCESS;
  }

  virtual void OnActivated(const rclcpp_lifecycle::State&) {}

  virtual void OnDeactivating(const rclcpp_lifecycle::State&) {}

  virtual void OnDeactivated(const rclcpp_lifecycle::State&) {}

  virtual void OnErroring(const rclcpp_lifecycle::State&) {}

  virtual void OnErrored(const rclcpp_lifecycle::State&) {}

  virtual void Process(std::shared_ptr<MsgT> msg) = 0;

  struct BasicStats {
    std::atomic<uint64_t> frame_in_{0};
    std::atomic<uint64_t> overwrite_drop_{0};

    void CountIn() { frame_in_.fetch_add(1, std::memory_order_relaxed); }
    void CountOverwriteDrop() { overwrite_drop_.fetch_add(1, std::memory_order_relaxed); }
    void Reset() {
      frame_in_.store(0, std::memory_order_relaxed);
      overwrite_drop_.store(0, std::memory_order_relaxed);
    }

    uint64_t FrameIn() const { return frame_in_.load(std::memory_order_relaxed); }
    uint64_t OverwriteDrop() const { return overwrite_drop_.load(std::memory_order_relaxed); }
  };

  virtual void OnMessagePostStore(bool overwritten) {
    stats_.CountIn();
    if (overwritten) {
      stats_.CountOverwriteDrop();
    }
  }

  virtual void OnWorkerStarted() {}

  void OnMessageInternal(typename MsgT::ConstSharedPtr msg) {
    const bool overwritten = latest_slot_.Store(std::move(msg));
    OnMessagePostStore(overwritten);
    cv_.notify_one();
  }

  LatestSlot<MsgT> latest_slot_;
  std::mutex cv_mutex_;
  std::condition_variable cv_;
  std::thread worker_thread_;
  std::atomic<bool> running_{false};

  BasicStats stats_;
  int worker_cpu_{-1};
  int worker_priority_{0};

 private:
  void StartWorker() {
    running_.store(true, std::memory_order_release);
    worker_thread_ = std::thread(&WorkerNode::WorkerLoop, this);
  }

  void StopWorker() {
    running_.store(false, std::memory_order_release);
    cv_.notify_all();

    if (worker_thread_.joinable()) {
      auto future = std::async(std::launch::async, [this]() { worker_thread_.join(); });

      if (future.wait_for(std::chrono::seconds(kWorkerJoinTimeoutSec))
          == std::future_status::timeout) {
        LOG_ERROR("worker thread join timeout after %ds, detaching (potential leak)",
                  kWorkerJoinTimeoutSec);
        worker_thread_.detach();
      }
    }
  }

  void WorkerLoop() {
    try {
      init_thread(worker_cpu_, SchedPolicy::kFifo, worker_priority_);
      LOG_INFO("worker cpu=%d prio=%d", worker_cpu_, worker_priority_);
    } catch (const std::exception& e) {
      LOG_WARN("worker setup failed: %s", e.what());
    }

    OnWorkerStarted();

    while (rclcpp::ok()) {
      {
        std::unique_lock<std::mutex> lock(cv_mutex_);
        cv_.wait(lock, [this]() {
          return !running_.load(std::memory_order_acquire) || latest_slot_.HasNew();
        });
      }

      if (!running_.load(std::memory_order_acquire)) {
        break;
      }

      bool had_new = false;
      auto msg = latest_slot_.TakeLatest(had_new);

      if (!had_new || !msg) {
        continue;
      }

      Process(msg);
    }
  }
};

}  // namespace perception_system
