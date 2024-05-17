#include "StreamIntrinsicsManager.hpp"
#include "logger/Logger.hpp"

namespace ob {
namespace core {

std::shared_ptr<StreamIntrinsicsManager> StreamIntrinsicsManager::getInstance() {
    static std::shared_ptr<StreamIntrinsicsManager> instance = std::shared_ptr<StreamIntrinsicsManager>(new StreamIntrinsicsManager());
    return instance;
}

StreamIntrinsicsManager::StreamIntrinsicsManager() {}

StreamIntrinsicsManager::~StreamIntrinsicsManager() noexcept {}

void StreamIntrinsicsManager::registerVideoStreamIntrinsics(std::shared_ptr<const StreamProfile> profile, const OBCameraIntrinsic &intrinsics) {
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

OBCameraIntrinsic StreamIntrinsicsManager::getVideoStreamIntrinsics(std::shared_ptr<const StreamProfile> profile) {
    if(!profile) {
        throw invalid_value_exception("Input stream profile is null.");
    }
    if(!profile->is<const VideoStreamProfile>()) {
        throw invalid_value_exception("Input stream profile is not a video stream profile.");
    }

    std::lock_guard<std::mutex> lock(mutex_);
    for(const auto& pair : videoStreamIntrinsics_) {
        if(pair.first.lock() == profile) {
            return pair.second;
        }
    }
    throw invalid_value_exception("Intrinsics for the input stream profile is not found.");
}

void StreamIntrinsicsManager::registerVideoStreamDistortion(std::shared_ptr<const StreamProfile> profile, const OBCameraDistortion &distortion) {
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

OBCameraDistortion StreamIntrinsicsManager::getVideoStreamDistortion(std::shared_ptr<const StreamProfile> profile) {
    if(!profile) {
        throw invalid_value_exception("Input stream profile is null.");
    }
    if(!profile->is<const VideoStreamProfile>()) {
        throw invalid_value_exception("Input stream profile is not a video stream profile.");
    }

    std::lock_guard<std::mutex> lock(mutex_);
    for(const auto& pair : videoStreamDistortion_) {
        if(pair.first.lock() == profile) {
            return pair.second;
        }
    }
    throw invalid_value_exception("Distortion for the input stream profile is not found.");
}

void StreamIntrinsicsManager::registerGyroStreamIntrinsics(std::shared_ptr<const StreamProfile> profile, const OBGyroIntrinsic &intrinsics) {
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

OBGyroIntrinsic StreamIntrinsicsManager::getGyroStreamIntrinsics(std::shared_ptr<const StreamProfile> profile) {
    if(!profile) {
        throw invalid_value_exception("Input stream profile is null.");
    }
    if(!profile->is<const GyroStreamProfile>()) {
        throw invalid_value_exception("Input stream profile is not a gyro stream profile.");
    }

    std::lock_guard<std::mutex> lock(mutex_);
    for(const auto& pair : gyroStreamIntrinsics_) {
        if(pair.first.lock() == profile) {
            return pair.second;
        }
    }
    throw invalid_value_exception("Intrinsics for the input stream profile is not found.");
}

void StreamIntrinsicsManager::registerAccelStreamIntrinsics(std::shared_ptr<const StreamProfile> profile, const OBAccelIntrinsic &intrinsics) {
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
            it++;
        }
    }

    accelStreamIntrinsics_.insert({ std::weak_ptr<const StreamProfile>(profile), intrinsics });
}

OBAccelIntrinsic StreamIntrinsicsManager::getAccelStreamIntrinsics(std::shared_ptr<const StreamProfile> profile) {
    if(!profile) {
        throw invalid_value_exception("Input stream profile is null.");
    }
    if(!profile->is<const AccelStreamProfile>()) {
        throw invalid_value_exception("Input stream profile is not a accel stream profile.");
    }

    std::lock_guard<std::mutex> lock(mutex_);
    for(const auto& pair : accelStreamIntrinsics_) {
        if(pair.first.lock() == profile) {
            return pair.second;
        }
    }
    throw invalid_value_exception("Intrinsics for the input stream profile is not found.");
}

}  // namespace core
}  // namespace ob