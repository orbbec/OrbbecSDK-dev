#pragma once

#include "Logger.hpp"
#include <spdlog/common.h>
#include <spdlog/fmt/fmt.h>
#include <spdlog/fmt/chrono.h>
#include <map>
#include <sstream>
#include <string>
#include <thread>
#include <mutex>
#include <ctime>

#define LOG_RECORD_MAP_SIZE_LIMIT 500  // 最大日志记录数
#define MAX_LOG_INTERVAL 60 * 1000     // 最大日志输出间隔 60000ms
#define DEF_MIN_LOG_INTVL 3000         // 默认最小日志输出间隔 3000ms
#define LOG_INTVL_OBJECT_TAG std::string(__FILE__) + std::to_string(__LINE__) + std::to_string((uint64_t)this)

struct ObLogIntvlRecord {
    uint32_t                              count;
    uint64_t                              interval;
    std::chrono::system_clock::time_point lastInvokeTime;
    std::chrono::system_clock::time_point lastLogTime;
    std::thread                           invokeThread;
    std::mutex                            mtx;
    std::condition_variable               cv;
    void                                  flush() {
        cv.notify_all();
        if(invokeThread.joinable()) {
            invokeThread.join();
        }
    }
    ~ObLogIntvlRecord() {
        flush();
    }
};

extern bool                                                     logIntvlRecordMapDestroyed;
extern std::map<std::string, std::shared_ptr<ObLogIntvlRecord>> logIntvlRecordMap;
extern std::mutex                                               logIntvlRecordMapMtx;

template <typename... Args>
void log_intvl_invoke(std::shared_ptr<ObLogIntvlRecord> record, uint64_t minIntvlMsec, spdlog::source_loc src_loc, spdlog::level::level_enum level,
                      std::string fmt, Args &&...args) {
    std::unique_lock<std::mutex> lock(record->mtx);
    record->cv.wait_for(lock, std::chrono::milliseconds(record->interval));
    if(record->count > 0) {
        auto     nowTime  = std::chrono::system_clock::now();
        uint64_t duration = std::chrono::duration_cast<std::chrono::milliseconds>(nowTime - record->lastInvokeTime).count();

        char   timestampStr[100];
        time_t timestamp = std::chrono::duration_cast<std::chrono::seconds>(record->lastLogTime.time_since_epoch()).count();
        std::strftime(timestampStr, sizeof(timestampStr), "%H:%M:%S", std::localtime(&timestamp));

        auto us    = std::chrono::duration_cast<std::chrono::microseconds>(record->lastLogTime.time_since_epoch()).count() % 1000000;
        auto usStr = fmt::format("{:06d}", us);

        fmt = fmt + " [**" + std::to_string(record->count) + " logs in " + std::to_string(duration) + "ms, last: " + timestampStr + "." + usStr.data() + "**]";
        spdlog::default_logger_raw()->log(src_loc, level, fmt, std::forward<Args>(args)...);

        uint64_t avgInterval = duration / record->count;
        if(avgInterval < record->interval) {  // 降低log输出频率
            record->interval *= 2;
            if(record->interval > MAX_LOG_INTERVAL) {
                record->interval = MAX_LOG_INTERVAL;
            }
        }
        else {
            avgInterval = minIntvlMsec;  // 恢复log输出频率
        }

        record->lastInvokeTime = nowTime;
        record->count          = 0;
    }
}

// 按时间间隔控制日志输出（log at intervals），当在interval时间内连续调用log_intvl时，只输出一条日志
// 连续频繁调用log_intvl时，日志输出频率会降低（间隔加长），直到达到最大时间间隔MAX_INTERVAL
template <typename... Args>
void log_intvl(std::shared_ptr<ObLogIntvlRecord> record, uint64_t minIntvlMsec, spdlog::source_loc src_loc, spdlog::level::level_enum level, std::string fmt,
               Args &&...args) {
    if(minIntvlMsec == 0) {
        spdlog::default_logger_raw()->log(src_loc, level, fmt, std::forward<Args>(args)...);
    }
    else {
        std::unique_lock<std::mutex> lock(record->mtx);
        auto                         nowTime = std::chrono::system_clock::now();
        record->lastLogTime                  = nowTime;
        record->count++;
        uint64_t duration           = std::chrono::duration_cast<std::chrono::milliseconds>(nowTime - record->lastInvokeTime).count();
        bool     zeroLastInvokeTime = record->lastInvokeTime.time_since_epoch() == std::chrono::system_clock::duration::zero();
        if(duration > record->interval || zeroLastInvokeTime) {
            if(!zeroLastInvokeTime) {
                uint64_t avgInterval = duration / record->count;
                if(avgInterval < record->interval) {  // 降低log输出频率
                    record->interval *= 2;
                    if(record->interval > MAX_LOG_INTERVAL) {
                        record->interval = MAX_LOG_INTERVAL;
                    }
                }
                else {
                    record->interval = minIntvlMsec;  // 恢复log输出频率
                }

                if(record->count > 1) {
                    fmt = fmt + " [**" + std::to_string(record->count) + " logs in " + std::to_string(duration) + "ms**]";
                }
            }

            spdlog::default_logger_raw()->log(src_loc, level, fmt, std::forward<Args>(args)...);
            record->count          = 0;
            record->lastInvokeTime = nowTime;
            lock.unlock();
            if(record->invokeThread.joinable())  // 通知线程退出(线程退出后会自动释放资源
            {
                record->cv.notify_all();
                record->invokeThread.join();
            }
        }
        else if(record->count == 1) {
            lock.unlock();
            if(record->invokeThread.joinable()) {
                record->invokeThread.join();
            }
            auto func            = std::bind(std::move(log_intvl_invoke<Args &...>), record, minIntvlMsec, src_loc, level, fmt, std::forward<Args>(args)...);
            record->invokeThread = std::thread(func);
        }
    }
}

// 控制日志输出时间间隔, 单位毫秒，0表示不控制
#define LOG_INTVL(tag, minIntvlMsec, level, ...)                                                                                         \
    do {                                                                                                                                 \
        std::unique_lock<std::mutex> lock(logIntvlRecordMapMtx);                                                                         \
        if(logIntvlRecordMapDestroyed) {                                                                                                 \
            break;                                                                                                                       \
        }                                                                                                                                \
        if(logIntvlRecordMap.size() > LOG_RECORD_MAP_SIZE_LIMIT) {                                                                       \
            LOG_WARN("logIntvlRecordMap size {} > {}, clear it!", logIntvlRecordMap.size(), LOG_RECORD_MAP_SIZE_LIMIT);                  \
            auto nowTime = std::chrono::system_clock::now();                                                                             \
            for(auto it = logIntvlRecordMap.begin(); it != logIntvlRecordMap.end();) {                                                   \
                uint64_t duration = std::chrono::duration_cast<std::chrono::milliseconds>(nowTime - it->second->lastInvokeTime).count(); \
                if(it->second->count == 0 && duration > it->second->interval) {                                                          \
                    it = logIntvlRecordMap.erase(it);                                                                                    \
                }                                                                                                                        \
                else {                                                                                                                   \
                    it++;                                                                                                                \
                }                                                                                                                        \
            }                                                                                                                            \
        }                                                                                                                                \
        auto iter = logIntvlRecordMap.find(tag);                                                                                         \
        if(iter == logIntvlRecordMap.end()) {                                                                                            \
            auto record      = std::make_shared<ObLogIntvlRecord>();                                                                     \
            record->count    = 0;                                                                                                        \
            record->interval = minIntvlMsec;                                                                                             \
            logIntvlRecordMap.insert(std::make_pair(tag, record));                                                                       \
            iter = logIntvlRecordMap.find(tag);                                                                                          \
        }                                                                                                                                \
        log_intvl(iter->second, minIntvlMsec, spdlog::source_loc{ __FILE__, __LINE__, SPDLOG_FUNCTION }, level, __VA_ARGS__);            \
    } while(0)

#define LOG_INTVL_ON_THREAD(minIntvlMsec, level, ...)                                  \
    do {                                                                               \
        std::stringstream ss;                                                          \
        ss << std::this_thread::get_id();                                              \
        std::string tag = std::string(__FILE__) + std::to_string(__LINE__) + ss.str(); \
        LOG_INTVL(tag, minIntvlMsec, level, __VA_ARGS__);                              \
    } while(0)

// LOG_XXX_INTVL 宏只能在类的成员函数中使用，因为它使用this指针作为tag（日志的时间间隔控制是绑定在具体对象上）
// LOG_XXX_INTVL_THREAD 宏使用当前线程id作为tag，可以在类的成员函数和普通函数中使用（日志的时间间隔控制绑定在具体线程上）
// 推荐优先使用 LOG_XXX_INTVL 宏，因为存在同一个对象的相同地方的日志输出是在不同线程调用的（如VideoSensor的原始帧数据回调）
#if SPDLOG_ACTIVE_LEVEL <= SPDLOG_LEVEL_TRACE
#define LOG_TRACE_INTVL(...) LOG_INTVL(LOG_INTVL_OBJECT_TAG, DEF_MIN_LOG_INTVL, spdlog::level::trace, __VA_ARGS__)
#define LOG_TRACE_INTVL_THREAD(...) LOG_INTVL_ON_THREAD(DEF_MIN_LOG_INTVL, spdlog::level::trace, __VA_ARGS__)
#define LOG_TRACE_INTVL_MS(minIntvlMsec, ...) LOG_INTVL(LOG_INTVL_OBJECT_TAG, minIntvlMsec, spdlog::level::trace, __VA_ARGS__)
#define LOG_TRACE_INTVL_THREAD_MS(minIntvlMsec, ...) LOG_INTVL_ON_THREAD(minIntvlMsec, spdlog::level::trace, __VA_ARGS__)
#else
#define LOG_TRACE_INTVL(...)
#define LOG_TRACE_INTVL_THREAD(...)
#define LOG_TRACE_INTVL_MS(...)
#define LOG_TRACE_INTVL_THREAD_MS(...)
#endif

#if SPDLOG_ACTIVE_LEVEL <= SPDLOG_LEVEL_DEBUG
#define LOG_DEBUG_INTVL(...) LOG_INTVL(LOG_INTVL_OBJECT_TAG, DEF_MIN_LOG_INTVL, spdlog::level::debug, __VA_ARGS__)
#define LOG_DEBUG_INTVL_THREAD(...) LOG_INTVL_ON_THREAD(DEF_MIN_LOG_INTVL, spdlog::level::debug, __VA_ARGS__)
#define LOG_DEBUG_INTVL_MS(minIntvlMsec, ...) LOG_INTVL(LOG_INTVL_OBJECT_TAG, minIntvlMsec, spdlog::level::debug, __VA_ARGS__)
#define LOG_DEBUG_INTVL_THREAD_MS(minIntvlMsec, ...) LOG_INTVL_ON_THREAD(minIntvlMsec, spdlog::level::debug, __VA_ARGS__)
#else
#define LOG_DEBUG_INTVL(...)
#define LOG_DEBUG_INTVL_THREAD(...)
#define LOG_DEBUG_INTVL_MS(...)
#define LOG_DEBUG_INTVL_THREAD_MS(...)
#endif

#if SPDLOG_ACTIVE_LEVEL <= SPDLOG_LEVEL_INFO
#define LOG_INFO_INTVL(...) LOG_INTVL(LOG_INTVL_OBJECT_TAG, DEF_MIN_LOG_INTVL, spdlog::level::info, __VA_ARGS__)
#define LOG_INFO_INTVL_THREAD(...) LOG_INTVL_ON_THREAD(DEF_MIN_LOG_INTVL, spdlog::level::info, __VA_ARGS__)
#define LOG_INFO_INTVL_MS(minIntvlMsec, ...) LOG_INTVL(LOG_INTVL_OBJECT_TAG, minIntvlMsec, spdlog::level::info, __VA_ARGS__)
#define LOG_INFO_INTVL_THREAD_MS(minIntvlMsec, ...) LOG_INTVL_ON_THREAD(minIntvlMsec, spdlog::level::info, __VA_ARGS__)
#else
#define LOG_INFO_INTVL(...)
#define LOG_INFO_INTVL_THREAD(...)
#define LOG_INFO_INTVL_MS(...)
#define LOG_INFO_INTVL_THREAD_MS(...)
#endif

#if SPDLOG_ACTIVE_LEVEL <= SPDLOG_LEVEL_WARN
#define LOG_WARN_INTVL(...) LOG_INTVL(LOG_INTVL_OBJECT_TAG, DEF_MIN_LOG_INTVL, spdlog::level::warn, __VA_ARGS__)
#define LOG_WARN_INTVL_THREAD(...) LOG_INTVL_ON_THREAD(DEF_MIN_LOG_INTVL, spdlog::level::warn, __VA_ARGS__)
#define LOG_WARN_INTVL_MS(minIntvlMsec, ...) LOG_INTVL(LOG_INTVL_OBJECT_TAG, minIntvlMsec, spdlog::level::warn, __VA_ARGS__)
#define LOG_WARN_INTVL_THREAD_MS(minIntvlMsec, ...) LOG_INTVL_ON_THREAD(minIntvlMsec, spdlog::level::warn, __VA_ARGS__)
#else
#define LOG_WARN_INTVL(...)
#define LOG_WARN_INTVL_THREAD(...)
#define LOG_WARN_INTVL_MS(...)
#define LOG_WARN_INTVL_THREAD_MS(...)
#endif

#if SPDLOG_ACTIVE_LEVEL <= SPDLOG_LEVEL_ERROR
#define LOG_ERROR_INTVL(...) LOG_INTVL(LOG_INTVL_OBJECT_TAG, DEF_MIN_LOG_INTVL, spdlog::level::err, __VA_ARGS__)
#define LOG_ERROR_INTVL_THREAD(...) LOG_INTVL_ON_THREAD(DEF_MIN_LOG_INTVL, spdlog::level::err, __VA_ARGS__)
#define LOG_ERROR_INTVL_MS(minIntvlMsec, ...) LOG_INTVL(LOG_INTVL_OBJECT_TAG, minIntvlMsec, spdlog::level::err, __VA_ARGS__)
#define LOG_ERROR_INTVL_THREAD_MS(minIntvlMsec, ...) LOG_INTVL_ON_THREAD(minIntvlMsec, spdlog::level::err, __VA_ARGS__)
#else
#define LOG_ERROR_INTVL(...)
#define LOG_ERROR_INTVL_THREAD(...)
#define LOG_ERROR_INTVL_MS(...)
#define LOG_ERROR_INTVL_THREAD_MS(...)
#endif

#if SPDLOG_ACTIVE_LEVEL <= SPDLOG_LEVEL_CRITICAL
#define LOG_FATAL_INTVL(...) LOG_INTVL(LOG_INTVL_OBJECT_TAG, DEF_MIN_LOG_INTVL, spdlog::level::critical, __VA_ARGS__)
#define LOG_FATAL_INTVL_THREAD(...) LOG_INTVL_ON_THREAD(DEF_MIN_LOG_INTVL, spdlog::level::critical, __VA_ARGS__)
#define LOG_FATAL_INTVL_MS(minIntvlMsec, ...) LOG_INTVL(LOG_INTVL_OBJECT_TAG, minIntvlMsec, spdlog::level::critical, __VA_ARGS__)
#define LOG_FATAL_INTVL_THREAD_MS(minIntvlMsec, ...) LOG_INTVL_ON_THREAD(minIntvlMsec, spdlog::level::critical, __VA_ARGS__)
#else
#define LOG_FATAL_INTVL(...)
#define LOG_FATAL_INTVL_THREAD(...)
#define LOG_FATAL_INTVL_MS(minIntvlMsec, ...)
#define LOG_FATAL_INTVL_THREAD_MS(minIntvlMsec, ...)
#endif