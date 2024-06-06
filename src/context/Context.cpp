#include "Context.hpp"
#include "utils/utils.hpp"

#include <mutex>

namespace libobsensor {

std::mutex             Context::instanceMutex_;
std::weak_ptr<Context> Context::instanceWeakPtr_;

std::shared_ptr<Context> Context::getInstance(const std::string& configPath) {
    std::unique_lock<std::mutex> lock(instanceMutex_);
    auto                         ctxInstance = instanceWeakPtr_.lock();
    if(!ctxInstance) {
        ctxInstance      = std::shared_ptr<Context>(new Context(configPath));
        instanceWeakPtr_ = ctxInstance;
    }
    return ctxInstance;
}

Context::Context(const std::string& configFilePath): logger_(Logger::getInstance()), deviceManager_(DeviceManager::getInstance()), frameMemoryPool_(FrameMemoryPool::getInstance()  ) {
    utils::unusedVar(configFilePath); // todo: use to load config file
}

Context::~Context() {}

}  // namespace libobsensor
