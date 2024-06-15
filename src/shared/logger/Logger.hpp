#pragma once

#ifdef _DEBUG
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#else
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG
#endif

#ifdef _WIN32
#ifndef SPDLOG_WCHAR_TO_UTF8_SUPPORT
#define SPDLOG_WCHAR_TO_UTF8_SUPPORT
#endif
#endif

// Debug trace information, only valid when compiled into debug version
#define LOG_TRACE(...) SPDLOG_LOGGER_TRACE(spdlog::default_logger(), __VA_ARGS__)

// Debugging information, for SDK developers, provides SDK internal running process and status information
#define LOG_DEBUG(...) SPDLOG_LOGGER_DEBUG(spdlog::default_logger(), __VA_ARGS__)

// General information, facing users, providing SDK running status and call result information
#define LOG_INFO(...) SPDLOG_LOGGER_INFO(spdlog::default_logger(), __VA_ARGS__)

// Warning information (such as: memory usage reaches the maximum limit, cache queue is full, system memory is about to be exhausted, etc.)
#define LOG_WARN(...) SPDLOG_LOGGER_WARN(spdlog::default_logger(), __VA_ARGS__)

// Error information (such as: user parameter error, device error calling sequence, device offline and unresponsive, etc.)
#define LOG_ERROR(...) SPDLOG_LOGGER_ERROR(spdlog::default_logger(), __VA_ARGS__)

// Fatal error message (for example: insufficient system memory prevents normal operation)
#define LOG_FATAL(...) SPDLOG_LOGGER_CRITICAL(spdlog::default_logger(), __VA_ARGS__)

#include <atomic>
#include <string>
#include <memory>
#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>
#include <openobsdk/h/ObTypes.h>
#include "utils/PublicTypeHelper.hpp"

namespace libobsensor {
typedef std::function<void(OBLogSeverity severity, const std::string &logMsg)> LogCallback;

void setGlobalLogSeverity(OBLogSeverity severity);
void setGlobalConsoleLogSeverity(OBLogSeverity severity);
void setGlobalFileLogConfig(OBLogSeverity severity, const std::string &directory = "", uint32_t maxFileSize = 0, uint32_t maxFileNum = 0);
void setGlobalFileLogDir(const std::string &directory);  // for api2
void setGlobalLogCallback(OBLogSeverity severity, LogCallback callback);

class Logger {
private:
    Logger();

    static std::mutex            instanceMutex_;
    static std::weak_ptr<Logger> instanceWeakPtr_;

public:
    static std::shared_ptr<Logger> getInstance();

    ~Logger() noexcept;

    // both console and file and callback log
    void setLogSeverity(OBLogSeverity severity);

    void setLogToConsole(OBLogSeverity severity);
    void setLogToFile(OBLogSeverity severity, const std::string &directory = "", uint32_t maxFileSize = 0, uint32_t maxFileNum = 0);
    void setLogToFileDir(const std::string &directory);
    void setLogCallback(OBLogSeverity severity, LogCallback callback);

    void createSinkFromGlobalConfig();
    void updateDefaultSpdLogger();

private:
    spdlog::sink_ptr consoleSink_;
    spdlog::sink_ptr fileSink_;
    spdlog::sink_ptr callbackSink_;

    std::shared_ptr<spdlog::details::registry> spdlogRegistry_;  // handle spdlog registry instance to control it's life cycle
};
}  // namespace libobsensor