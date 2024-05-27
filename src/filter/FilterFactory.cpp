#include "FilterFactory.hpp"
#include "utils/PublicTypeHelper.hpp"

#include <mutex>

namespace libobsensor {

static std::mutex                     instanceMutex;
static std::shared_ptr<FilterFactory> instance;

std::shared_ptr<FilterFactory> FilterFactory::getInstance() {
    std::unique_lock<std::mutex> lk(instanceMutex);
    if(!instance) {
        instance = std::shared_ptr<FilterFactory>(new FilterFactory());
    }
    return instance;
}

void FilterFactory::destroyInstance() {
    std::unique_lock<std::mutex> lk(instanceMutex);
    instance.reset();
}

FilterFactory::FilterFactory() {}

FilterFactory::~FilterFactory() noexcept {}

std::shared_ptr<IFilter> FilterFactory::createPublicFilter(const std::string &filterName) {
    // todo: implement
    utils::unusedVar(filterName);
    return nullptr;
}

std::shared_ptr<IFilter> FilterFactory::createPrivateFilter(const std::string &filterName, const std::string &activationKey) {
    // todo: implement
    utils::unusedVar(filterName);
    utils::unusedVar(activationKey);
    return nullptr;
}

}  // namespace libobsensor