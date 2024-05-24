#pragma once
#include "openobsdk/h/ObTypes.h"

#include "frame/Frame.hpp"
#include "stream/StreamProfile.hpp"
#include "ISensor.hpp"
#include "IDevice.hpp"

#include "utils/StringUtils.hpp"

namespace libobsensor {
class Context;
}

#ifdef __cplusplus
extern "C" {
#endif

struct ob_frame {
    std::shared_ptr<libobsensor::Context> ctx;
    std::shared_ptr<libobsensor::Frame>   frame;
    std::atomic<int>                      refCnt = 1;
};

struct ob_stream_profile {
    std::shared_ptr<libobsensor::Context>             ctx;
    std::shared_ptr<const libobsensor::StreamProfile> profile;
};

struct ob_sensor {
    std::shared_ptr<libobsensor::Context> ctx;
    std::shared_ptr<libobsensor::IDevice> device;
    OBSensorType                          type;  // 不直接创建sensor，而是使用用时才从device中获取（懒加载）
};

struct ob_device {
    std::shared_ptr<libobsensor::Context> ctx;
    std::shared_ptr<libobsensor::IDevice> device;
};

void translate_exception(const char *name, std::string args, ob_error **result);

#ifdef __cplusplus
}
#endif

#define BEGIN_API_CALL try
#define NOEXCEPT_RETURN(...)                                             \
    catch(...) {                                                         \
        std::ostringstream ss;                                           \
        libobsensor::utils::ArgsToStream(ss, #__VA_ARGS__, __VA_ARGS__); \
        return translate_exception(__FUNCTION__, ss.str());              \
    }
#define HANDLE_EXCEPTIONS_NO_RETURN(...)                                 \
    catch(...) {                                                         \
        std::ostringstream ss;                                           \
        libobsensor::utils::ArgsToStream(ss, #__VA_ARGS__, __VA_ARGS__); \
        translate_exception(__FUNCTION__, ss.str(), error);              \
    }
#define NO_ARGS_HANDLE_EXCEPTIONS_NO_RETURN(...)      \
    catch(...) {                                      \
        translate_exception(__FUNCTION__, "", error); \
    }
#define HANDLE_EXCEPTIONS_AND_RETURN(R, ...)                             \
    catch(...) {                                                         \
        std::ostringstream ss;                                           \
        libobsensor::utils::ArgsToStream(ss, #__VA_ARGS__, __VA_ARGS__); \
        translate_exception(__FUNCTION__, ss.str(), error);              \
        return R;                                                        \
    }
#define NO_ARGS_HANDLE_EXCEPTIONS_AND_RETURN(R)       \
    catch(...) {                                      \
        translate_exception(__FUNCTION__, "", error); \
        return R;                                     \
    }
