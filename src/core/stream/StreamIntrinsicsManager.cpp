#include "StreamIntrinsicsManager.hpp"
#include "logger/Logger.hpp"

namespace libobsensor {

std::mutex                             StreamIntrinsicsManager::instanceMutex_;
std::weak_ptr<StreamIntrinsicsManager> StreamIntrinsicsManager::instanceWeakPtr_;

std::shared_ptr<StreamIntrinsicsManager> StreamIntrinsicsManager::getInstance() {
    std::unique_lock<std::mutex> lock(instanceMutex_);
    auto                         instance = instanceWeakPtr_.lock();
    if(!instance) {
        instance         = std::shared_ptr<StreamIntrinsicsManager>(new StreamIntrinsicsManager());
        instanceWeakPtr_ = instance;
    }
    return instance;
}

StreamIntrinsicsManager::StreamIntrinsicsManager() {}

StreamIntrinsicsManager::~StreamIntrinsicsManager() noexcept = default;

void StreamIntrinsicsManager::registerVideoStreamIntrinsics(const std::shared_ptr<const StreamProfile>& profile, const OBCameraIntrinsic &intrinsics) {
    if(!profile) {
        throw invalid_value_exception("Input stream profile is null.");
    }
    if(!profile->is<const VideoStreamProfile>()) {
        throw invalid_value_exception("Input stream profile is not a video stream profile.");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    // remove expired stream profiles
    for(auto it = videoStreamIntrinsics_.begin(); it != videoStreamIntrinsics_.end();) {
        if(it->first.expired()) {
            it = videoStreamIntrinsics_.erase(it);
        }
        else {
            it++;
        }
    }
    videoStreamIntrinsics_.insert({ std::weak_ptr<const StreamProfile>(profile), intrinsics });
}

OBCameraIntrinsic StreamIntrinsicsManager::getVideoStreamIntrinsics(const std::shared_ptr<const StreamProfile>& profile) {
    if(!profile) {
        throw invalid_value_exception("Input stream profile is null.");
    }
    if(!profile->is<const VideoStreamProfile>()) {
        throw invalid_value_exception("Input stream profile is not a video stream profile.");
    }

    std::lock_guard<std::mutex> lock(mutex_);
    for(const auto &pair: videoStreamIntrinsics_) {
        if(pair.first.lock() == profile) {
            return pair.second;
        }
    }
    throw invalid_value_exception("Intrinsics for the input stream profile is not found.");
}

void StreamIntrinsicsManager::registerVideoStreamDistortion(const std::shared_ptr<const StreamProfile>& profile, const OBCameraDistortion &distortion) {
    if(!profile) {
        throw invalid_value_exception("Input stream profile is null.");
    }
    if(!profile->is<const VideoStreamProfile>()) {
        throw invalid_value_exception("Input stream profile is not a video stream profile.");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    // remove expired stream profiles
    for(auto it = videoStreamDistortion_.begin(); it != videoStreamDistortion_.end();) {
        if(it->first.expired()) {
            it = videoStreamDistortion_.erase(it);
        }
        else {
            it++;
        }
    }

    videoStreamDistortion_.insert({ std::weak_ptr<const StreamProfile>(profile), distortion });
}

OBCameraDistortion StreamIntrinsicsManager::getVideoStreamDistortion(const std::shared_ptr<const StreamProfile>& profile) {
    if(!profile) {
        throw invalid_value_exception("Input stream profile is null.");
    }
    if(!profile->is<const VideoStreamProfile>()) {
        throw invalid_value_exception("Input stream profile is not a video stream profile.");
    }

    std::lock_guard<std::mutex> lock(mutex_);
    for(const auto &pair: videoStreamDistortion_) {
        if(pair.first.lock() == profile) {
            return pair.second;
        }
    }
    throw invalid_value_exception("Distortion for the input stream profile is not found.");
}

void StreamIntrinsicsManager::registerGyroStreamIntrinsics(const std::shared_ptr<const StreamProfile>& profile, const OBGyroIntrinsic &intrinsics) {
    if(!profile) {
        throw invalid_value_exception("Input stream profile is null.");
    }
    if(!profile->is<const GyroStreamProfile>()) {
        throw invalid_value_exception("Input stream profile is not a gyro stream profile.");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    // remove expired stream profiles
    for(auto it = gyroStreamIntrinsics_.begin(); it != gyroStreamIntrinsics_.end();) {
        if(it->first.expired()) {
            it = gyroStreamIntrinsics_.erase(it);
        }
        else {
            it++;
        }
    }

    gyroStreamIntrinsics_.insert({ std::weak_ptr<const StreamProfile>(profile), intrinsics });
}

OBGyroIntrinsic StreamIntrinsicsManager::getGyroStreamIntrinsics(const std::shared_ptr<const StreamProfile>& profile) {
    if(!profile) {
        throw invalid_value_exception("Input stream profile is null.");
    }
    if(!profile->is<const GyroStreamProfile>()) {
        throw invalid_value_exception("Input stream profile is not a gyro stream profile.");
    }

    std::lock_guard<std::mutex> lock(mutex_);
    for(const auto &pair: gyroStreamIntrinsics_) {
        if(pair.first.lock() == profile) {
            return pair.second;
        }
    }
    throw invalid_value_exception("Intrinsics for the input stream profile is not found.");
}

void StreamIntrinsicsManager::registerAccelStreamIntrinsics(const std::shared_ptr<const StreamProfile>& profile, const OBAccelIntrinsic &intrinsics) {
    if(!profile) {
        throw invalid_value_exception("Input stream profile is null.");
    }
    if(!profile->is<const AccelStreamProfile>()) {
        throw invalid_value_exception("Input stream profile is not a accel stream profile.");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    // remove expired stream profiles
    for(auto it = accelStreamIntrinsics_.begin(); it != accelStreamIntrinsics_.end();) {
        if(it->first.expired()) {
            it = accelStreamIntrinsics_.erase(it);
        }
        else {
            ++it;
        }
    }

    accelStreamIntrinsics_.insert({ std::weak_ptr<const StreamProfile>(profile), intrinsics });
}

OBAccelIntrinsic StreamIntrinsicsManager::getAccelStreamIntrinsics(const std::shared_ptr<const StreamProfile>& profile) {
    if(!profile) {
        throw invalid_value_exception("Input stream profile is null.");
    }
    if(!profile->is<const AccelStreamProfile>()) {
        throw invalid_value_exception("Input stream profile is not a accel stream profile.");
    }

    std::lock_guard<std::mutex> lock(mutex_);
    for(const auto &pair: accelStreamIntrinsics_) {
        if(pair.first.lock() == profile) {
            return pair.second;
        }
    }
    throw invalid_value_exception("Intrinsics for the input stream profile is not found.");
}

}  // namespace libobsensor