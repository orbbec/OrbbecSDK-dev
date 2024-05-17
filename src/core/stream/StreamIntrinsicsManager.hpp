#pragma once

#include "openobsdk/h/ObTypes.h"
#include "StreamProfile.hpp"
#include <map>
#include <memory>
#include <vector>

namespace ob {
namespace core {

struct weak_stream_profile_compare {
    bool operator()(const std::weak_ptr<const StreamProfile> &a, const std::weak_ptr<const StreamProfile> &b) const {
        auto sharedA = a.lock();
        auto sharedB = b.lock();

        if(sharedA && sharedB) {
            return sharedA < sharedB;
        }

        return sharedA != nullptr;
    }
};

class StreamIntrinsicsManager {
public:
    static std::shared_ptr<StreamIntrinsicsManager> getInstance();
    ~StreamIntrinsicsManager() noexcept;

    void               registerVideoStreamIntrinsics(std::shared_ptr<const StreamProfile> profile, const OBCameraIntrinsic &intrinsics);
    OBCameraIntrinsic  getVideoStreamIntrinsics(std::shared_ptr<const StreamProfile> profile);
    void               registerVideoStreamDistortion(std::shared_ptr<const StreamProfile> profile, const OBCameraDistortion &distortion);
    OBCameraDistortion getVideoStreamDistortion(std::shared_ptr<const StreamProfile> profile);
    void               registerGyroStreamIntrinsics(std::shared_ptr<const StreamProfile> profile, const OBGyroIntrinsic &intrinsics);
    OBGyroIntrinsic    getGyroStreamIntrinsics(std::shared_ptr<const StreamProfile> profile);
    void               registerAccelStreamIntrinsics(std::shared_ptr<const StreamProfile> profile, const OBAccelIntrinsic &intrinsics);
    OBAccelIntrinsic   getAccelStreamIntrinsics(std::shared_ptr<const StreamProfile> profile);

private:
    StreamIntrinsicsManager();

private:
    std::map<std::weak_ptr<const StreamProfile>, OBCameraIntrinsic, weak_stream_profile_compare>  videoStreamIntrinsics_;
    std::map<std::weak_ptr<const StreamProfile>, OBCameraDistortion, weak_stream_profile_compare> videoStreamDistortion_;
    std::map<std::weak_ptr<const StreamProfile>, OBGyroIntrinsic, weak_stream_profile_compare>    gyroStreamIntrinsics_;
    std::map<std::weak_ptr<const StreamProfile>, OBAccelIntrinsic, weak_stream_profile_compare>   accelStreamIntrinsics_;

    std::mutex mutex_;
};

}  // namespace core
}  // namespace ob