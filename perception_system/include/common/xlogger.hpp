#pragma once

#include <atomic>
#include <cstdint>
#include <cstdio>

inline constexpr char kAnsiReset[] = "\033[0m";
inline constexpr char kAnsiDefault[] = "";
inline constexpr char kAnsiRed[] = "\033[31m";
inline constexpr char kAnsiYellow[] = "\033[33m";
inline constexpr char kAnsiCyan[] = "\033[36m";

enum class LogLevel { kDebug = 0, kInfo = 1, kWarn = 2, kError = 3, kFatal = 4 };

#ifndef ACTIVE_LOG_LEVEL
#define ACTIVE_LOG_LEVEL LogLevel::kInfo
#endif

constexpr const char* FileBaseName(const char* path) {
  const char* base = path;
  for (const char* p = path; *p; ++p) {
    if (*p == '/' || *p == '\\') {
      base = p + 1;
    }
  }
  return base;
}

#if defined(__FILE_NAME__)
#define XLOG_FILENAME __FILE_NAME__
#else
#define XLOG_FILENAME FileBaseName(__FILE__)
#endif

#if (defined(LOG_BACKEND_STDIO) + defined(LOG_BACKEND_SPDLOG) + defined(LOG_BACKEND_RCLCPP)) > 1
#error "Select only one logging backend."
#elif !defined(LOG_BACKEND_STDIO) && !defined(LOG_BACKEND_SPDLOG) && !defined(LOG_BACKEND_RCLCPP)
#define LOG_BACKEND_STDIO
#endif

#if defined(LOG_BACKEND_STDIO)

inline void LogWrite(const char* color,
                     const char* label,
                     const char* file,
                     int line,
                     const char* func,
                     const char* msg) {
  flockfile(stdout);
  printf("%s[%s][%s:%d %s] %s%s\n", color, label, file, line, func, msg, kAnsiReset);
  funlockfile(stdout);
}

#elif defined(LOG_BACKEND_SPDLOG)

#include <spdlog/spdlog.h>

inline void LogWrite(const char* /*color*/,
                     const char* label,
                     const char* file,
                     int line,
                     const char* func,
                     const char* msg) {
  const auto loc = spdlog::source_loc{file, line, func};
  switch (label[0]) {
    case 'D':
      spdlog::log(loc, spdlog::level::debug, msg);
      break;
    case 'I':
      spdlog::log(loc, spdlog::level::info, msg);
      break;
    case 'W':
      spdlog::log(loc, spdlog::level::warn, msg);
      break;
    case 'E':
      spdlog::log(loc, spdlog::level::err, msg);
      break;
    default:
      spdlog::log(loc, spdlog::level::critical, msg);
      break;
  }
}

#elif defined(LOG_BACKEND_RCLCPP)

#include <rcutils/logging.h>

#include <rclcpp/rclcpp.hpp>

inline rclcpp::Logger& GetRclcppLogger() {
  static rclcpp::Logger logger = rclcpp::get_logger("system");
  return logger;
}

inline void SetRclcppLogger(const rclcpp::Logger& logger) {
  GetRclcppLogger() = logger;
}

inline void LogWrite(const char* /*color*/,
                     const char* label,
                     const char* file,
                     int line,
                     const char* func,
                     const char* msg) {
  const rcutils_log_location_t loc{func, file, static_cast<size_t>(line)};
  int severity = RCUTILS_LOG_SEVERITY_FATAL;
  switch (label[0]) {
    case 'D':
      severity = RCUTILS_LOG_SEVERITY_DEBUG;
      break;
    case 'I':
      severity = RCUTILS_LOG_SEVERITY_INFO;
      break;
    case 'W':
      severity = RCUTILS_LOG_SEVERITY_WARN;
      break;
    case 'E':
      severity = RCUTILS_LOG_SEVERITY_ERROR;
      break;
    default:
      break;
  }
  rcutils_log(&loc, severity, GetRclcppLogger().get_name(), "%s", msg);
}

#else
#error "Select exactly one logging backend."
#endif

template <LogLevel kLevel>
inline void LogDispatch(const char* file, int line, const char* func, const char* msg) {
  if constexpr (kLevel < ACTIVE_LOG_LEVEL) {
    return;
  }

  if constexpr (kLevel == LogLevel::kDebug) {
    LogWrite(kAnsiCyan, "D", file, line, func, msg);
  } else if constexpr (kLevel == LogLevel::kInfo) {
    LogWrite(kAnsiDefault, "I", file, line, func, msg);
  } else if constexpr (kLevel == LogLevel::kWarn) {
    LogWrite(kAnsiYellow, "W", file, line, func, msg);
  } else if constexpr (kLevel == LogLevel::kError) {
    LogWrite(kAnsiRed, "E", file, line, func, msg);
  } else {
    LogWrite(kAnsiRed, "F", file, line, func, msg);
  }
}

#define XLOG_IMPL(level, fmt, ...)                                              \
  do {                                                                          \
    char log_buf[512];                                                          \
    const int fmt_len = snprintf(log_buf, sizeof(log_buf), fmt, ##__VA_ARGS__); \
    if (fmt_len < 0) {                                                          \
      snprintf(log_buf, sizeof(log_buf), "<log formatting error>");             \
    } else if (static_cast<size_t>(fmt_len) >= sizeof(log_buf)) {               \
      constexpr const char kSuffix[] = "... [truncated]";                       \
      constexpr size_t kSuffixLen = sizeof(kSuffix) - 1;                        \
      const size_t suffix_pos = sizeof(log_buf) - kSuffixLen - 1;               \
      snprintf(log_buf + suffix_pos, kSuffixLen + 1, "%s", kSuffix);            \
    }                                                                           \
    LogDispatch<level>(XLOG_FILENAME, __LINE__, __func__, log_buf);             \
  } while (0)

#define XLOG_THROTTLE_IMPL(level, clock, interval_ms, fmt, ...)                          \
  do {                                                                                   \
    static std::atomic<int64_t> last_ns{0};                                              \
    const int64_t now_ns = (clock).now().nanoseconds();                                  \
    const int64_t interval_ns = static_cast<int64_t>(interval_ms) * 1000000ll;           \
    int64_t prev_ns = last_ns.load(std::memory_order_relaxed);                           \
    if (interval_ns <= 0 || prev_ns == 0 || now_ns - prev_ns >= interval_ns) {           \
      if (last_ns.compare_exchange_strong(prev_ns, now_ns, std::memory_order_relaxed)) { \
        XLOG_IMPL(level, fmt, ##__VA_ARGS__);                                            \
      }                                                                                  \
    }                                                                                    \
  } while (0)

#define LOG_DEBUG(fmt, ...) XLOG_IMPL(LogLevel::kDebug, fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...) XLOG_IMPL(LogLevel::kInfo, fmt, ##__VA_ARGS__)
#define LOG_WARN(fmt, ...) XLOG_IMPL(LogLevel::kWarn, fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) XLOG_IMPL(LogLevel::kError, fmt, ##__VA_ARGS__)
#define LOG_FATAL(fmt, ...) XLOG_IMPL(LogLevel::kFatal, fmt, ##__VA_ARGS__)

#define LOG_INFO_THROTTLE(clock, interval_ms, fmt, ...) \
  XLOG_THROTTLE_IMPL(LogLevel::kInfo, clock, interval_ms, fmt, ##__VA_ARGS__)
#define LOG_WARN_THROTTLE(clock, interval_ms, fmt, ...) \
  XLOG_THROTTLE_IMPL(LogLevel::kWarn, clock, interval_ms, fmt, ##__VA_ARGS__)
#define LOG_ERROR_THROTTLE(clock, interval_ms, fmt, ...) \
  XLOG_THROTTLE_IMPL(LogLevel::kError, clock, interval_ms, fmt, ##__VA_ARGS__)
