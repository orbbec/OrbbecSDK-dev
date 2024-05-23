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

#define LOG_TRACE(...) SPDLOG_LOGGER_TRACE(spdlog::default_logger(), __VA_ARGS__)  // 调试跟踪信息, 仅编译成debug版本时有效
#define LOG_DEBUG(...) SPDLOG_LOGGER_DEBUG(spdlog::default_logger(), __VA_ARGS__)  // 调试信息, 面向SDK开发人员，提供SDK内部运行过程及状态信息
#define LOG_INFO(...) SPDLOG_LOGGER_INFO(spdlog::default_logger(), __VA_ARGS__)  // 一般信息，面相用户，提供SDK的运行状态及调用结果信息
#define LOG_WARN(...) SPDLOG_LOGGER_WARN(spdlog::default_logger(), __VA_ARGS__)  // 警告信息（如：内存占用到达最大限定值，缓存队列占满，系统内存即将耗尽等）
#define LOG_ERROR(...) SPDLOG_LOGGER_ERROR(spdlog::default_logger(), __VA_ARGS__)  // 错误信息（如：用户参数错误，设备错误调用循序，设备掉线无响应等）
#define LOG_FATAL(...) SPDLOG_LOGGER_CRITICAL(spdlog::default_logger(), __VA_ARGS__)  // 致命错误信息（如：系统内存不足导致无法正常运行）

#include <atomic>
#include <string>
#include <memory>
#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>
#include <openobsdk/h/ObTypes.h>

namespace libobsensor{
typedef std::function<void(OBLogSeverity severity, const std::string &logMsg)> LogCallback;

void setGlobalLogSeverity(OBLogSeverity severity);
void setGlobalConsoleLogSeverity(OBLogSeverity severity);
void setGlobalFileLogConfig(OBLogSeverity severity, const std::string &directory = "", uint32_t maxFileSize = 0, uint32_t maxFileNum = 0);
void setGlobalFileLogDir(const std::string &directory);  // for api2
void setGlobalLogCallback(OBLogSeverity severity, LogCallback callback);

class Logger {
public:
    Logger();
    ~Logger() noexcept;

    // both console and file and callback log
    void setLogSeverity(OBLogSeverity severity);

    void setLogToConsole(OBLogSeverity severity);
    void setLogToFile(OBLogSeverity severity, const std::string &directory = "", uint32_t maxFileSize = 0, uint32_t maxFileNum = 0);
    void setLogToFileDir(const std::string &directory);
    void setLogCallback(OBLogSeverity severity, LogCallback callback);

    // void flush();

private:
    inline void createSinkFromGlobalConfig();
    inline void updateDefaultSpdLogger();

private:
    spdlog::sink_ptr consoleSink_;
    spdlog::sink_ptr fileSink_;
    spdlog::sink_ptr callbackSink_;

    std::shared_ptr<spdlog::details::registry> spdlogRegistry_;  // handle spdlog registry instance to control it's life cycle
};
}  // namespace ob