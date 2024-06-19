#pragma once
#include <memory>
#include <string>
#include <mutex>
#include <memory>

#include "devicemanager/DeviceManager.hpp"
#include "logger/Logger.hpp"
#include "frame/FrameMemoryPool.hpp"
#include "stream/StreamIntrinsicsManager.hpp"
#include "stream/StreamExtrinsicsManager.hpp"

namespace libobsensor {
class Context {
private:
    explicit Context(const std::string &configFilePath = "");

    static std::mutex             instanceMutex_;
    static std::weak_ptr<Context> instanceWeakPtr_;

public:
    ~Context() noexcept;

    static std::shared_ptr<Context> getInstance(const std::string &configPath = "");
    static bool                     hasInstance();

    std::shared_ptr<DeviceManager>   getDeviceManager();
    std::shared_ptr<Logger>          getLogger() const;
    std::shared_ptr<FrameMemoryPool> getFrameMemoryPool() const;

private:
    std::shared_ptr<DeviceManager>   deviceManager_;
    std::shared_ptr<Logger>          logger_;
    std::shared_ptr<FrameMemoryPool> frameMemoryPool_;
    std::shared_ptr<StreamIntrinsicsManager>     streamIntrinsicsManager_;
    std::shared_ptr<StreamExtrinsicsManager>     streamExtrinsicsManager_;
};
}  // namespace libobsensor

#ifdef __cplusplus
extern "C" {
#endif
struct ob_context_t {
    std::shared_ptr<libobsensor::Context> context;
};
#ifdef __cplusplus
}
#endif