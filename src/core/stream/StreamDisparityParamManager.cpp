#include "StreamDisparityParamManager.hpp"

namespace libobsensor {

std::mutex                                 StreamDisparityParamManager::instanceMutex_;
std::weak_ptr<StreamDisparityParamManager> StreamDisparityParamManager::instanceWeakPtr_;

std::shared_ptr<StreamDisparityParamManager> StreamDisparityParamManager::getInstance() {
    std::unique_lock<std::mutex> lock(instanceMutex_);
    auto                         instance = instanceWeakPtr_.lock();
    if(!instance) {
        instance         = std::shared_ptr<StreamDisparityParamManager>(new StreamDisparityParamManager());
        instanceWeakPtr_ = instance;
    }
    return instance;
}

StreamDisparityParamManager::StreamDisparityParamManager() {}

StreamDisparityParamManager::~StreamDisparityParamManager() noexcept = default;

void StreamDisparityParamManager::registerDisparityProcessParam(std::shared_ptr<const StreamProfile> profile, const OBDisparityProcessParam &param) {
    if(!profile) {
        throw invalid_value_exception("Input stream profile is null.");
    }
    if(!profile->is<const StructuredLightStreamProfile>()) {
        throw invalid_value_exception("Input stream profile is not a disparity stream profile.");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    // remove expired stream profiles
    for(auto it = disparityProcessParamMap_.begin(); it != disparityProcessParamMap_.end();) {
        if(it->first.expired()) {
            it = disparityProcessParamMap_.erase(it);
        }
        else {
            it++;
        }
    }

    disparityProcessParamMap_.insert({ std::weak_ptr<const StreamProfile>(profile), param });
}

OBDisparityProcessParam StreamDisparityParamManager::getVideoStreamDisparityParam(std::shared_ptr<const StreamProfile> profile) {
    if(!profile) {
        throw invalid_value_exception("Input stream profile is null.");
    }
    if(!profile->is<const StructuredLightStreamProfile>()) {
        throw invalid_value_exception("Input stream profile is not a disparity stream profile.");
    }

    std::lock_guard<std::mutex> lock(mutex_);
    for(const auto &pair: disparityProcessParamMap_) {
        if(pair.first.lock() == profile) {
            return pair.second;
        }
    }
    throw invalid_value_exception("disparityparam for the input stream profile is not found.");
}


}  // namespace libobsensor