#include "Context.hpp"
#include "utils/Utils.hpp"

#include <mutex>

namespace libobsensor {

std::mutex             Context::instanceMutex_;
std::weak_ptr<Context> Context::instanceWeakPtr_;

std::shared_ptr<Context> Context::getInstance(const std::string &configPath) {
    std::unique_lock<std::mutex> lock(instanceMutex_);
    auto                         ctxInstance = instanceWeakPtr_.lock();
    if(!ctxInstance) {
        ctxInstance      = std::shared_ptr<Context>(new Context(configPath));
        instanceWeakPtr_ = ctxInstance;
    }
    return ctxInstance;
}

bool Context::hasInstance() {
    std::unique_lock<std::mutex> lock(instanceMutex_);
    return !instanceWeakPtr_.expired();
}

Context::Context(const std::string &configFilePath) {
    // Perform initialization here for sequential order.
    logger_          = Logger::getInstance();
    frameMemoryPool_ = FrameMemoryPool::getInstance();
    streamIntrinsicsManager_     = StreamIntrinsicsManager::getInstance();
    streamExtrinsicsManager_     = StreamExtrinsicsManager::getInstance();

    utils::unusedVar(configFilePath);  // todo: use to load config file
}

Context::~Context() noexcept {}

std::shared_ptr<DeviceManager> Context::getDeviceManager() {
    if(deviceManager_ == nullptr){
        deviceManager_ = DeviceManager::getInstance();
    }
    return deviceManager_;
}

std::shared_ptr<Logger> Context::getLogger() const {
    return logger_;
}

std::shared_ptr<FrameMemoryPool> Context::getFrameMemoryPool() const {
    return frameMemoryPool_;
}

}  // namespace libobsensor
