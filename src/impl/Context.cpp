#include "ImplTypes.hpp"
#include "openobsdk/h/Context.h"
#include "context/Context.hpp"

#ifdef __cplusplus
extern "C" {
#endif

ob_context *ob_create_context(ob_error **error) BEGIN_API_CALL {
    auto ctx      = libobsensor::Context::getInstance();
    auto impl     = new ob_context();
    impl->context = ctx;
    return impl;
}
NO_ARGS_HANDLE_EXCEPTIONS_AND_RETURN(nullptr)

ob_context *ob_create_context_with_config(const char *config_path, ob_error **error) BEGIN_API_CALL {
    auto ctx      = libobsensor::Context::getInstance(config_path);
    auto impl     = new ob_context();
    impl->context = ctx;
    return impl;
}
NO_ARGS_HANDLE_EXCEPTIONS_AND_RETURN(nullptr)

void ob_delete_context(ob_context *context, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(context);
    delete context;
}
HANDLE_EXCEPTIONS_NO_RETURN(context)

ob_device_list *ob_query_device_list(ob_context *context, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(context);
    auto deviceMgr    = context->context->getDeviceManager();
    auto devListImpl  = new ob_device_list();
    devListImpl->list = deviceMgr->getDeviceInfoList();
    return devListImpl;
}
HANDLE_EXCEPTIONS_AND_RETURN(nullptr, context)

void ob_enable_net_device_enumeration(ob_context *context, bool enable, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(context);
    auto deviceMgr = context->context->getDeviceManager();
    deviceMgr->enableNetDeviceEnumeration(enable);
}
HANDLE_EXCEPTIONS_NO_RETURN(context, enable)

ob_device *ob_create_net_device(ob_context *context, const char *address, uint16_t port, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(context);
    auto deviceMgr  = context->context->getDeviceManager();
    auto device     = deviceMgr->createNetDevice(address, port);
    auto devImpl    = new ob_device();
    devImpl->device = device;
    return devImpl;
}
HANDLE_EXCEPTIONS_AND_RETURN(nullptr, context, address, port)

void ob_set_device_changed_callback(ob_context *context, ob_device_changed_callback callback, void *user_data, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(context);
    auto deviceMgr = context->context->getDeviceManager();
    deviceMgr->setDeviceChangedCallback([callback, user_data](std::vector<std::shared_ptr<const libobsensor::IDeviceEnumInfo>> removed,
                                                              std::vector<std::shared_ptr<const libobsensor::IDeviceEnumInfo>> added) {
        auto removedImpl  = new ob_device_list();
        removedImpl->list = removed;
        auto addedImpl    = new ob_device_list();
        addedImpl->list   = added;
        callback(removedImpl, addedImpl, user_data);
    });
}
HANDLE_EXCEPTIONS_NO_RETURN(context, callback, user_data)

void ob_enable_device_clock_sync(ob_context *context, uint64_t repeatInterval, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(context);
    auto deviceMgr = context->context->getDeviceManager();
    deviceMgr->enableDeviceClockSync(repeatInterval);
}
HANDLE_EXCEPTIONS_NO_RETURN(context, repeatInterval)

void ob_free_idle_memory(ob_context *context, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(context);
    auto frameMemPool = context->context->getFrameMemoryPool();
    frameMemPool->freeIdleMemory();
}
HANDLE_EXCEPTIONS_NO_RETURN(context)

void ob_set_logger_severity(ob_log_severity severity, ob_error **error) BEGIN_API_CALL {
    if(libobsensor::Context::hasInstance()) {
        auto context = libobsensor::Context::getInstance();
        auto logger  = context->getLogger();
        logger->setLogSeverity(severity);
    }
    else {
        libobsensor::setGlobalLogSeverity(severity);
    }
}
HANDLE_EXCEPTIONS_NO_RETURN(severity)

void ob_set_logger_to_file(ob_log_severity severity, const char *directory, ob_error **error) BEGIN_API_CALL {
    if(libobsensor::Context::hasInstance()) {
        auto context = libobsensor::Context::getInstance();
        auto logger  = context->getLogger();
        logger->setLogToFile(severity, directory);
    }
    else {
        libobsensor::setGlobalFileLogConfig(severity, directory);
    }
}
HANDLE_EXCEPTIONS_NO_RETURN(severity, directory)

void ob_set_logger_callback(ob_log_severity severity, ob_log_callback callback, void *user_data, ob_error **error) BEGIN_API_CALL {
    if(libobsensor::Context::hasInstance()) {
        auto context = libobsensor::Context::getInstance();
        auto logger  = context->getLogger();
        logger->setLogCallback(severity, [callback, user_data](ob_log_severity severity, const std::string logMsg) {  //
            callback(severity, logMsg.c_str(), user_data);
        });
    }
    else {
        libobsensor::setGlobalLogCallback(severity, [callback, user_data](ob_log_severity severity, const std::string logMsg) {  //
            callback(severity, logMsg.c_str(), user_data);
        });
    }
}
HANDLE_EXCEPTIONS_NO_RETURN(severity, callback, user_data)

void ob_set_logger_to_console(ob_log_severity severity, ob_error **error) BEGIN_API_CALL {
    if(libobsensor::Context::hasInstance()) {
        auto context = libobsensor::Context::getInstance();
        auto logger  = context->getLogger();
        logger->setLogToConsole(severity);
    }
    else {
        libobsensor::setGlobalConsoleLogSeverity(severity);
    }
}
HANDLE_EXCEPTIONS_NO_RETURN(severity)

#ifdef __cplusplus
}
#endif