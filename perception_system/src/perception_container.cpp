#include <atomic>
#include <csignal>
#include <thread>
#include <vector>

#include "common/affinity.hpp"
#include "common/xlogger.hpp"
#include "perception_system/camera_component.hpp"
#include "perception_system/detector_component.hpp"
#include "perception_system/image_proc_component.hpp"
#include "perception_system/lifecycle_manager.hpp"
#include "perception_system/visualize_component.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/executors/static_single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {
std::atomic<bool> g_shutdown{false};

void signal_handler(int) {
  g_shutdown.store(true, std::memory_order_relaxed);
}
}  // namespace

static void do_shutdown(perception_system::LifecycleManager* mgr,
                        rclcpp::Executor& ec,
                        rclcpp::Executor& ei,
                        rclcpp::Executor& ed,
                        rclcpp::Executor& ev,
                        rclcpp::Executor& em,
                        std::vector<std::thread>& threads) {
  if (mgr) mgr->shutdown();
  ec.cancel();
  ei.cancel();
  ed.cancel();
  ev.cancel();
  em.cancel();
  for (auto& t : threads) {
    if (t.joinable()) t.join();
  }
  rclcpp::shutdown();
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  rclcpp::NodeOptions opts;
  opts.use_intra_process_comms(true);
  opts.automatically_declare_parameters_from_overrides(true);

  auto cam = std::make_shared<perception_system::CameraComponent>(opts);
  auto proc = std::make_shared<perception_system::ImageProcComponent>(opts);
  auto det = std::make_shared<perception_system::DetectorComponent>(opts);
  auto viz = std::make_shared<perception_system::VisualizeComponent>(opts);

  auto mgr = std::make_shared<perception_system::LifecycleManager>(opts);

  auto exec_camera = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto exec_imgproc = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto exec_detector = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto exec_viz = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  exec_camera->add_node(cam->get_node_base_interface());
  exec_imgproc->add_node(proc->get_node_base_interface());
  exec_detector->add_node(det->get_node_base_interface());
  exec_viz->add_node(viz->get_node_base_interface());

  auto exec_manager = std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();
  exec_manager->add_node(mgr);

  struct ThreadConfig {
    std::shared_ptr<rclcpp::Executor> exec;
    int cpu;
    perception_system::SchedPolicy policy;
    int priority;
    const char* name;
  };

  const std::vector<ThreadConfig> configs = {
      {exec_camera, 1, perception_system::SchedPolicy::kFifo, 60, "camera_exec"},
      {exec_imgproc, 2, perception_system::SchedPolicy::kNormal, 0, "imgproc_exec"},
      {exec_detector, 3, perception_system::SchedPolicy::kNormal, 0, "detector_exec"},
      {exec_viz, 4, perception_system::SchedPolicy::kNormal, 0, "viz_exec"},
      {exec_manager, 0, perception_system::SchedPolicy::kNormal, 0, "mgr_exec"},
  };

  std::vector<std::thread> threads;
  threads.reserve(configs.size());

  for (const auto& cfg : configs) {
    threads.emplace_back([cfg]() {
      try {
        perception_system::init_thread(cfg.cpu, cfg.policy, cfg.priority);
        LOG_INFO("[container] %s : CPU %d  priority=%d", cfg.name, cfg.cpu, cfg.priority);
      } catch (const std::exception& e) {
        LOG_WARN("[container] %s: %s; fallback: SCHED_OTHER", cfg.name, e.what());
      }
      cfg.exec->spin();
    });
  }

  if (!mgr->startup()) {
    LOG_FATAL("startup failed");
    do_shutdown(mgr.get(),
                *exec_camera,
                *exec_imgproc,
                *exec_detector,
                *exec_viz,
                *exec_manager,
                threads);
    return 1;
  }

  while (rclcpp::ok() && !g_shutdown.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  do_shutdown(mgr.get(),
              *exec_camera,
              *exec_imgproc,
              *exec_detector,
              *exec_viz,
              *exec_manager,
              threads);

  LOG_INFO("[container] clean shutdown");
  return 0;
}
