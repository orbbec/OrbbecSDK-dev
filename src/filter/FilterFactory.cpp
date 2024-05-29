#include "FilterFactory.hpp"
#include "public_filters/publicFilterLoader.hpp"
#include "private_filters/PrivFilterLoader.hpp"
#include "utils/PublicTypeHelper.hpp"
#include "exception/ObException.hpp"

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

FilterFactory::FilterFactory() {
    auto publicFilterCreators  = PublicFilterCreatorLoader::getCreators();
    auto privateFilterCreators = PrivFilterCreatorLoader::getCreators();
    filterCreators_.insert(publicFilterCreators.begin(), publicFilterCreators.end());
    filterCreators_.insert(privateFilterCreators.begin(), privateFilterCreators.end());
}

FilterFactory::~FilterFactory() noexcept {}

std::shared_ptr<IFilter> FilterFactory::createFilter(const std::string &filterName) {
    auto iter = filterCreators_.find(filterName);
    if(iter == filterCreators_.end()) {
        throw invalid_value_exception("Invalid filter name, cannot find filter creator for filter name: " + filterName);
    }
    return iter->second->create();
}

std::shared_ptr<IFilter> FilterFactory::createPrivateFilter(const std::string &filterName, const std::string &activationKey) {
    auto iter = filterCreators_.find(filterName);
    if(iter == filterCreators_.end()) {
        throw invalid_value_exception("Invalid filter name, cannot find filter creator for filter name: " + filterName);
    }
    auto privFilterCreator = std::dynamic_pointer_cast<IPrivFilterCreator>(iter->second);
    if(!privFilterCreator) {
        LOG_WARN("Filter name {} is not a private filter, dose not need activation key to create filter", filterName);
        return iter->second->create();
    }
    return privFilterCreator->create(activationKey);
}

std::shared_ptr<IFilterCreator> FilterFactory::getFilterCreator(const std::string &filterName) {
    auto iter = filterCreators_.find(filterName);
    if(iter == filterCreators_.end()) {
        throw invalid_value_exception("Invalid filter name, cannot find filter creator for filter name: " + filterName);
    }
    return iter->second;
}

}  // namespace libobsensor