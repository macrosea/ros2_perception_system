#pragma once

#include <pthread.h>
#include <sched.h>

#include <stdexcept>
#include <string>

namespace perception_system {

inline void bind_this_thread(int cpu) {
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(cpu, &cpuset);

  int ret = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);

  if (ret != 0) {
    throw std::runtime_error("bind_this_thread: pthread_setaffinity_np failed, cpu="
                             + std::to_string(cpu) + " errno=" + std::to_string(ret));
  }
}

enum class SchedPolicy {
  kNormal,
  kFifo,
  kRr,
};

inline void set_thread_priority(SchedPolicy policy, int priority) {
  int linux_policy{};
  switch (policy) {
    case SchedPolicy::kNormal:
      linux_policy = SCHED_OTHER;
      break;
    case SchedPolicy::kFifo:
      linux_policy = SCHED_FIFO;
      break;
    case SchedPolicy::kRr:
      linux_policy = SCHED_RR;
      break;
  }

  sched_param param{};
  param.sched_priority = (linux_policy == SCHED_OTHER) ? 0 : priority;

  int ret = pthread_setschedparam(pthread_self(), linux_policy, &param);
  if (ret != 0) {
    throw std::runtime_error(
        "set_thread_priority: pthread_setschedparam failed"
        ", policy="
        + std::to_string(linux_policy) + " priority=" + std::to_string(param.sched_priority)
        + " errno=" + std::to_string(ret) + " (need CAP_SYS_NICE or rtprio in limits.conf)");
  }
}

inline void init_thread(int cpu, SchedPolicy policy, int priority) {
  bind_this_thread(cpu);
  set_thread_priority(policy, priority);
}

}  // namespace perception_system
