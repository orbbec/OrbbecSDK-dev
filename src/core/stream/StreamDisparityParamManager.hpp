#pragma once
#include "openobsdk/h/ObTypes.h"
#include "StreamProfile.hpp"
#include <map>
#include <memory>
#include <vector>
#include <mutex>

namespace libobsensor {
struct StreamProfilePtrCompare {
    bool operator()(const std::weak_ptr<const StreamProfile> &a, const std::weak_ptr<const StreamProfile> &b) const {
        auto sharedA = a.lock();
        auto sharedB = b.lock();

        if(sharedA && sharedB) {
            return sharedA < sharedB;
        }

        return sharedA != nullptr;
    }
};

class StreamDisparityParamManager {
private:
    StreamDisparityParamManager();

    static std::mutex                                 instanceMutex_;
    static std::weak_ptr<StreamDisparityParamManager> instanceWeakPtr_;

public:
    static std::shared_ptr<StreamDisparityParamManager> getInstance();
    ~StreamDisparityParamManager() noexcept;
    void                    registerDisparityProcessParam(std::shared_ptr<const StreamProfile> profile, const OBDisparityProcessParam &param);
    OBDisparityProcessParam getVideoStreamDisparityParam(std::shared_ptr<const StreamProfile> profile);

private:
    std::mutex                                                                                         mutex_;
    std::map<std::weak_ptr<const StreamProfile>, OBDisparityProcessParam, StreamProfilePtrCompare> disparityProcessParamMap_;
};

}  // namespace libobsensor