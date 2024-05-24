#include "Context.hpp"

#include <mutex>

namespace libobsensor {

std::mutex                    ContextInstanceMutex;
static std::weak_ptr<Context> ContextInstanceWeakPtr;
const int                     ConfigStandardVerMajor = 1;
const int                     ConfigStandardVerMinor = 1;

Context::Context(std::string configFilePath) {}

Context::~Context() {}

std::shared_ptr<Context> Context::getInstance(std::string configPath) {
    std::unique_lock<std::mutex> lock(ContextInstanceMutex);
    auto                         ctxInstance = ContextInstanceWeakPtr.lock();
    if(!ctxInstance) {
        // Since the Context constructor is private, the Context constructor cannot be accessed within the shared_ptr constructor, so new is required first.
        ctxInstance            = std::shared_ptr<Context>(new Context(configPath));
        ContextInstanceWeakPtr = ctxInstance;
    }
    return ctxInstance;
}

}  // namespace libobsensor
