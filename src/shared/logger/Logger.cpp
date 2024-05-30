#include "Logger.hpp"
#include "LoggerHelper.hpp"
#include "LoggerInterval.hpp"

#include "exception/ObException.hpp"

#include "LogCallbackSink.hpp"
#ifdef __ANDROID__
#include <spdlog/sinks/android_sink.h>
#else
#include <spdlog/sinks/stdout_color_sinks.h>
#endif
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/async_logger.h>
#include <spdlog/async.h>
#include "spdlog/pattern_formatter.h"

#include <map>

std::map<std::string, std::shared_ptr<ObLogIntvlRecord>> logIntvlRecordMap;
std::mutex                                               logIntvlRecordMapMtx;
std::atomic<bool>                                        logIntvlRecordMapDestroyed(false);

namespace libobsensor {

const std::map<OBLogSeverity, spdlog::level::level_enum> OBLogSeverityToSpdlogLevel = {
#ifdef _DEBUG
    { OB_LOG_SEVERITY_DEBUG, spdlog::level::level_enum::trace },
#else
    { OB_LOG_SEVERITY_DEBUG, spdlog::level::level_enum::debug },
#endif
    { OB_LOG_SEVERITY_INFO, spdlog::level::level_enum::info },   { OB_LOG_SEVERITY_WARN, spdlog::level::level_enum::warn },
    { OB_LOG_SEVERITY_ERROR, spdlog::level::level_enum::err },   { OB_LOG_SEVERITY_FATAL, spdlog::level::level_enum::critical },
    { OB_LOG_SEVERITY_OFF, spdlog::level::level_enum::off },
};

const std::map<spdlog::level::level_enum, OBLogSeverity> SpdlogLevelToOBLogSeverity = {
    { spdlog::level::level_enum::trace, OB_LOG_SEVERITY_DEBUG }, { spdlog::level::level_enum::debug, OB_LOG_SEVERITY_DEBUG },
    { spdlog::level::level_enum::info, OB_LOG_SEVERITY_INFO },   { spdlog::level::level_enum::warn, OB_LOG_SEVERITY_WARN },
    { spdlog::level::level_enum::err, OB_LOG_SEVERITY_ERROR },   { spdlog::level::level_enum::critical, OB_LOG_SEVERITY_FATAL },
    { spdlog::level::level_enum::off, OB_LOG_SEVERITY_OFF },
};

#ifdef __ANDROID__
const char *OB_DEFAULT_LOG_FILE_PATH = "/sdcard/Orbbec/Log/";
#else
const char *OB_DEFAULT_LOG_FILE_PATH = "Log/";
#endif

const OBLogSeverity OB_DEFAULT_LOG_SEVERITY  = OB_LOG_SEVERITY_INFO;
const std::string   OB_DEFAULT_LOG_FMT       = "[%m/%d %H:%M:%S.%f][%l][%t][%s:%#] %v";
const uint64_t      OB_DEFAULT_MAX_FILE_SIZE = 1024 * 1024 * 100;
const uint16_t      OB_DEFAULT_MAX_FILE_NUM  = 3;
const std::string   OB_DEFAULT_LOG_FILE_NAME = "OrbbecSDK.log.txt";

struct {
    bool          loadFileLogSeverityFromXmlConfig = true;
    OBLogSeverity fileLogSeverity                  = OB_LOG_SEVERITY_DEBUG;

    bool        loadFileLogPathFromXmlConfig = true;
    std::string fileLogOutputDir             = OB_DEFAULT_LOG_FILE_PATH;
    std::string fileLogFileName              = OB_DEFAULT_LOG_FILE_NAME;
    uint64_t    fileLogMaxFileSize           = OB_DEFAULT_MAX_FILE_SIZE;
    uint64_t    fileLogMaxFileNum            = OB_DEFAULT_MAX_FILE_NUM;

    bool          loadConsoleLogSeverityFromXmlConfig = true;
    OBLogSeverity consoleLogSeverity                  = OB_DEFAULT_LOG_SEVERITY;

    bool          loadCallbackLogSeverityFromXmlConfig = true;
    OBLogSeverity callbackLogSeverity                  = OB_DEFAULT_LOG_SEVERITY;
    LogCallback   logCallback                          = nullptr;

    bool async = false;
} GlobalLoggerConfig;

void setGlobalLogSeverity(OBLogSeverity severity) {
    GlobalLoggerConfig.loadFileLogSeverityFromXmlConfig    = false;
    GlobalLoggerConfig.fileLogSeverity                     = severity;
    GlobalLoggerConfig.loadConsoleLogSeverityFromXmlConfig = false;
    GlobalLoggerConfig.consoleLogSeverity                  = severity;
}

void setGlobalConsoleLogSeverity(OBLogSeverity severity) {
    GlobalLoggerConfig.loadConsoleLogSeverityFromXmlConfig = false;
    GlobalLoggerConfig.consoleLogSeverity                  = severity;
}

void setGlobalFileLogConfig(OBLogSeverity severity, const std::string &directory, uint32_t maxFileSize, uint32_t maxFileNum) {
    GlobalLoggerConfig.loadFileLogSeverityFromXmlConfig = false;
    GlobalLoggerConfig.fileLogSeverity                  = severity;
    GlobalLoggerConfig.loadFileLogPathFromXmlConfig     = false;
    GlobalLoggerConfig.fileLogOutputDir                 = directory;
    GlobalLoggerConfig.fileLogMaxFileSize               = maxFileSize * 1024 * 1024;  // MB to Byte
    GlobalLoggerConfig.fileLogMaxFileNum                = maxFileNum;
    if(directory.empty()) {
        GlobalLoggerConfig.fileLogOutputDir = OB_DEFAULT_LOG_FILE_PATH;
    }
    if(maxFileSize == 0) {
        GlobalLoggerConfig.fileLogMaxFileSize = OB_DEFAULT_MAX_FILE_SIZE;
    }
    if(maxFileNum == 0) {
        GlobalLoggerConfig.fileLogMaxFileNum = OB_DEFAULT_MAX_FILE_NUM;
    }
}

void setGlobalFileLogDir(const std::string &directory) {
    GlobalLoggerConfig.loadFileLogPathFromXmlConfig = false;
    GlobalLoggerConfig.fileLogOutputDir             = directory;
    if(directory.empty()) {
        GlobalLoggerConfig.fileLogOutputDir = OB_DEFAULT_LOG_FILE_PATH;
    }
}

void setGlobalLogCallback(OBLogSeverity severity, LogCallback logCallback) {
    GlobalLoggerConfig.loadCallbackLogSeverityFromXmlConfig = false;
    GlobalLoggerConfig.callbackLogSeverity                  = severity;
    GlobalLoggerConfig.logCallback                          = logCallback;
}

spdlog::sink_ptr createFileSink() {
    spdlog::sink_ptr fileSink;
    if(GlobalLoggerConfig.fileLogSeverity != OB_LOG_SEVERITY_OFF) {
        auto  path     = GlobalLoggerConfig.fileLogOutputDir + "/" + GlobalLoggerConfig.fileLogFileName;
        auto &fileSize = GlobalLoggerConfig.fileLogMaxFileSize;
        auto &fileNum  = GlobalLoggerConfig.fileLogMaxFileNum;
        try {
            fileSink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(path, fileSize, fileNum);
        }
        catch(const std::exception &e) {
            LOG_ERROR("Error creating file sink for logger! {}", e.what());
            fileSink = nullptr;
        }
        catch(...) {
            LOG_ERROR("Unknown error creating file sink for logger! path:{}", path);
            fileSink = nullptr;
        }
        if(fileSink) {
            auto fileLogLevel = OBLogSeverityToSpdlogLevel.find(GlobalLoggerConfig.fileLogSeverity)->second;
            fileSink->set_level(fileLogLevel);
        }
    }
    return fileSink;
}

void Logger::createSinkFromGlobalConfig() {
    if(GlobalLoggerConfig.consoleLogSeverity != OB_LOG_SEVERITY_OFF) {
        if(!consoleSink_) {
#ifdef __ANDROID__
            consoleSink_ = std::make_shared<spdlog::sinks::android_sink_mt>("OrbbecSDK");
#else
            consoleSink_ = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
#endif
        }
        auto consoleLogLevel = OBLogSeverityToSpdlogLevel.find(GlobalLoggerConfig.consoleLogSeverity)->second;
        consoleSink_->set_level(consoleLogLevel);
    }

    if(GlobalLoggerConfig.fileLogSeverity != OB_LOG_SEVERITY_OFF) {
        if(fileSink_) {
            fileSink_->flush();
            fileSink_.reset();
        }
        fileSink_ = createFileSink();
    }

    if(GlobalLoggerConfig.logCallback != nullptr && GlobalLoggerConfig.callbackLogSeverity != OB_LOG_SEVERITY_OFF) {
        if(callbackSink_) {
            callbackSink_->flush();
            callbackSink_.reset();
        }
        callbackSink_         = std::make_shared<spdlog::sinks::callback_sink_mt>([](spdlog::level::level_enum logLevel, std::string msg) {
            if(GlobalLoggerConfig.logCallback) {
                GlobalLoggerConfig.logCallback(SpdlogLevelToOBLogSeverity.find(logLevel)->second, msg);
            }
        });
        auto callbackLogLevel = OBLogSeverityToSpdlogLevel.find(GlobalLoggerConfig.callbackLogSeverity)->second;
        callbackSink_->set_level(callbackLogLevel);
    }
}
void Logger::updateDefaultSpdLogger() {
    std::vector<spdlog::sink_ptr> sinks;
    if(consoleSink_) {
        sinks.push_back(consoleSink_);
    }
    if(fileSink_) {
        sinks.push_back(fileSink_);
    }
    if(callbackSink_) {
        sinks.push_back(callbackSink_);
    }

    std::shared_ptr<spdlog::logger> spdLogger;
    if(GlobalLoggerConfig.async) {
        spdlog::init_thread_pool(1024, 1);  // queue with 1k items and 1 threads, multiple threads will cause the log output order to be disordered

        // Asynchronous logger
        spdLogger = std::make_shared<spdlog::async_logger>("OrbbecSDK", sinks.begin(), sinks.end(),  //
                                                           spdlog::thread_pool(), spdlog::async_overflow_policy::block);

        spdlog::flush_every(std::chrono::seconds(1));  // Set to flush the log every 1 second
    }
    else {
        // Synchronize logger
        spdLogger = std::make_shared<spdlog::logger>("OrbbecSDK", sinks.begin(), sinks.end());
    }

    spdlog::set_default_logger(spdLogger);
    spdlog::set_level(spdlog::level::trace);  // Set the logger log level (here set to trace, the actual output will be output according to the sink's log
                                              // level)
    spdlog::flush_on(spdlog::level::trace);   // Set the flush log level (immediately output logs when receiving logs greater than or equal to this level)
    spdlog::set_pattern(OB_DEFAULT_LOG_FMT);
}

std::mutex              Logger::instanceMutex_;
std::weak_ptr<Logger>   Logger::instanceWeakPtr_;
std::shared_ptr<Logger> Logger::getInstance() {
    std::lock_guard<std::mutex> lock(instanceMutex_);
    auto                        instance = instanceWeakPtr_.lock();
    if(!instance) {
        instance         = std::shared_ptr<Logger>(new Logger());
        instanceWeakPtr_ = instance;
    }
    return instance;
}

Logger::Logger() : spdlogRegistry_(spdlog::details::registry::instance_ptr()) {
    spdlog::set_pattern(OB_DEFAULT_LOG_FMT);

    std::lock_guard<std::mutex> lock(logIntvlRecordMapMtx);
    logIntvlRecordMapDestroyed = false;

    createSinkFromGlobalConfig();
    updateDefaultSpdLogger();
}

Logger::~Logger() noexcept {
    {
        std::lock_guard<std::mutex> lock(logIntvlRecordMapMtx);
        logIntvlRecordMapDestroyed = true;
        for(auto &it: logIntvlRecordMap) {
            it.second->flush();
            it.second.reset();
        }
        logIntvlRecordMap.clear();
    }

    spdlog::set_default_logger(std::make_shared<spdlog::logger>("EmptySinksLogger"));

    if(consoleSink_) {
        consoleSink_->flush();
        consoleSink_.reset();
    }
    if(fileSink_) {
        fileSink_->flush();
        fileSink_.reset();
    }
    if(callbackSink_) {
        callbackSink_->flush();
        callbackSink_.reset();
    }

    spdlogRegistry_.reset();
}

void Logger::setLogSeverity(OBLogSeverity severity) {
    GlobalLoggerConfig.loadFileLogSeverityFromXmlConfig     = false;
    GlobalLoggerConfig.fileLogSeverity                      = severity;
    GlobalLoggerConfig.loadConsoleLogSeverityFromXmlConfig  = false;
    GlobalLoggerConfig.consoleLogSeverity                   = severity;
    GlobalLoggerConfig.loadCallbackLogSeverityFromXmlConfig = false;
    GlobalLoggerConfig.callbackLogSeverity                  = severity;

    if(severity == OB_LOG_SEVERITY_OFF) {
        if(consoleSink_) {
            consoleSink_->flush();
            consoleSink_.reset();
        }
        if(fileSink_) {
            fileSink_->flush();
            fileSink_.reset();
        }
        if(callbackSink_) {
            callbackSink_->flush();
            callbackSink_.reset();
        }
        updateDefaultSpdLogger();
    }
    else {
        createSinkFromGlobalConfig();
        updateDefaultSpdLogger();
    }
}

void Logger::setLogToConsole(OBLogSeverity severity) {
    GlobalLoggerConfig.consoleLogSeverity = severity;
    spdlog::level::level_enum spdLogLevel = OBLogSeverityToSpdlogLevel.find(severity)->second;
    if(consoleSink_) {
        if(severity == OB_LOG_SEVERITY_OFF) {
            consoleSink_->flush();
            consoleSink_.reset();
            updateDefaultSpdLogger();
        }
        else {
            consoleSink_->set_level(spdLogLevel);
        }
    }
    else if(severity != OB_LOG_SEVERITY_OFF) {
#ifdef __ANDROID__
        consoleSink_ = std::make_shared<spdlog::sinks::android_sink_mt>("OrbbecSDK");
#else
        consoleSink_ = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
#endif
        consoleSink_->set_level(spdLogLevel);
        updateDefaultSpdLogger();
    }
}

void Logger::setLogToFile(OBLogSeverity severity, const std::string &directory, uint32_t maxFileSize, uint32_t maxFileNum) {
    GlobalLoggerConfig.fileLogSeverity = severity;
    if(!directory.empty()) {
        GlobalLoggerConfig.fileLogOutputDir = directory;
    }

    if(maxFileSize > 0) {
        GlobalLoggerConfig.fileLogMaxFileSize = maxFileSize * 1024 * 1024;  // MB to Byte
    }

    if(maxFileNum > 0) {
        GlobalLoggerConfig.fileLogMaxFileNum = maxFileNum;
    }

    if(fileSink_) {
        fileSink_->flush();
        fileSink_.reset();
    }
    fileSink_ = createFileSink();
    updateDefaultSpdLogger();
};

void Logger::setLogToFileDir(const std::string &directory) {
    if(!directory.empty()) {
        GlobalLoggerConfig.fileLogOutputDir = directory;
    }
    if(fileSink_) {
        fileSink_->flush();
        fileSink_.reset();
    }
    fileSink_ = createFileSink();
    updateDefaultSpdLogger();
}

void Logger::setLogCallback(OBLogSeverity severity, LogCallback callback) {
    GlobalLoggerConfig.callbackLogSeverity = severity;
    GlobalLoggerConfig.logCallback         = callback;

    if(callbackSink_) {
        callbackSink_->flush();
        callbackSink_.reset();
    }

    if(severity != OB_LOG_SEVERITY_OFF && callback) {
        callbackSink_         = std::make_shared<spdlog::sinks::callback_sink_mt>([callback](spdlog::level::level_enum logLevel, std::string msg) {
            if(callback) {
                callback(SpdlogLevelToOBLogSeverity.find(logLevel)->second, msg);
            }
        });
        auto callbackLogLevel = OBLogSeverityToSpdlogLevel.find(GlobalLoggerConfig.callbackLogSeverity)->second;
        callbackSink_->set_level(callbackLogLevel);
    }
    updateDefaultSpdLogger();
}

}  // namespace libobsensor