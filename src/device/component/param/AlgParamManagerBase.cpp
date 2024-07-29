#include "AlgParamManagerBase.hpp"
#include "property/InternalProperty.hpp"
#include "stream/StreamIntrinsicsManager.hpp"
#include "stream/StreamExtrinsicsManager.hpp"
#include "stream/StreamProfileFactory.hpp"
#include "exception/ObException.hpp"

#include <vector>
#include <sstream>
#include <utility>

namespace libobsensor {

bool findBestMatchedD2CProfile(const std::vector<OBD2CProfile> &d2cProfileList, const std::shared_ptr<const VideoStreamProfile> &profile,
                               OBD2CProfile &result) {
    bool found = false;
    // match same resolution
    for(auto &d2cProfile: d2cProfileList) {
        // using software align profile
        if(d2cProfile.alignType != ALIGN_D2C_SW && d2cProfile.alignType != ALIGN_D2C_HW_SW_BOTH) {
            continue;
        }

        auto streamType = profile->getType();
        if((streamType == OB_STREAM_DEPTH || streamType == OB_STREAM_IR || streamType == OB_STREAM_IR_LEFT || streamType == OB_STREAM_IR_RIGHT)
           && static_cast<uint32_t>(d2cProfile.depthWidth) == profile->getWidth() && static_cast<uint32_t>(d2cProfile.depthWidth) == profile->getHeight()) {
            found  = true;
            result = d2cProfile;
            break;
        }
        else if(streamType == OB_STREAM_COLOR && static_cast<uint32_t>(d2cProfile.colorWidth) == profile->getWidth()
                && static_cast<uint32_t>(d2cProfile.colorHeight) == profile->getHeight()) {
            found  = true;
            result = d2cProfile;
            break;
        }
    }

    if(!found) {
        // match same ratio
        float ratio = (float)profile->getWidth() / profile->getHeight();
        for(auto &d2cProfile: d2cProfileList) {
            // using software align profile
            if(d2cProfile.alignType != ALIGN_D2C_SW && d2cProfile.alignType != ALIGN_D2C_HW_SW_BOTH) {
                continue;
            }

            auto streamType = profile->getType();
            if((streamType == OB_STREAM_DEPTH || streamType == OB_STREAM_IR || streamType == OB_STREAM_IR_LEFT || streamType == OB_STREAM_IR_RIGHT)
               && (float)d2cProfile.depthWidth / d2cProfile.depthHeight == ratio) {
                found  = true;
                result = d2cProfile;
                break;
            }
            else if(streamType == OB_STREAM_COLOR && (float)d2cProfile.colorWidth / d2cProfile.colorHeight == ratio) {
                found  = true;
                result = d2cProfile;
                break;
            }
        }
    }

    return found;
}

AlgParamManagerBase::AlgParamManagerBase(IDevice *owner) : DeviceComponentBase(owner) {}

void AlgParamManagerBase::bindStreamProfileParams(std::vector<std::shared_ptr<const StreamProfile>> streamProfileList) {
    bindExtrinsic(streamProfileList);
    bindIntrinsic(streamProfileList);
}

const std::vector<OBD2CProfile> &AlgParamManagerBase::getD2CProfileList() const {
    return d2cProfileList_;
}

const std::vector<OBCameraParam> &AlgParamManagerBase::getCalibrationCameraParamList() const {
    return calibrationCameraParamList_;
}

const OBIMUCalibrateParams &AlgParamManagerBase::getIMUCalibrationParam() const {
    return imuCalibParam_;
}

void AlgParamManagerBase::bindExtrinsic(std::vector<std::shared_ptr<const StreamProfile>> streamProfileList) {
    auto extrinsicMgr = StreamExtrinsicsManager::getInstance();
    for(auto &sp: streamProfileList) {
        auto basicSpIter = std::find_if(basicStreamProfileList_.begin(), basicStreamProfileList_.end(), [&](const auto &basicSp) {  //
            return basicSp->getType() == sp->getType();
        });
        if(basicSpIter == basicStreamProfileList_.end()) {
            throw libobsensor::unsupported_operation_exception("Can not find basic stream profile to bind extrinsic!");
        }
        extrinsicMgr->registerSameExtrinsics(sp, *basicSpIter);
    }
}

void AlgParamManagerBase::bindIntrinsic(std::vector<std::shared_ptr<const StreamProfile>> streamProfileList) {
    auto intrinsicMgr = StreamIntrinsicsManager::getInstance();
    bool mirrored     = false;

    {
        auto owner      = getOwner();
        auto propServer = owner->getPropertyServer();
        if(propServer->isPropertySupported(OB_PROP_DEPTH_MIRROR_MODULE_STATUS_BOOL, PROP_OP_READ, PROP_ACCESS_INTERNAL)) {
            mirrored = propServer->getPropertyValueT<bool>(OB_PROP_DEPTH_MIRROR_MODULE_STATUS_BOOL);
        }
    }

    for(const auto &sp: streamProfileList) {
        if(sp->is<AccelStreamProfile>()) {
            const auto &imuCalibParam = getIMUCalibrationParam();
            intrinsicMgr->registerAccelStreamIntrinsics(sp, imuCalibParam.singleIMUParams[0].acc);
        }
        else if(sp->is<GyroStreamProfile>()) {
            const auto &imuCalibParam = getIMUCalibrationParam();
            intrinsicMgr->registerGyroStreamIntrinsics(sp, imuCalibParam.singleIMUParams[0].gyro);
        }
        else {
            OBCameraIntrinsic  intrinsic  = { 0 };
            OBCameraDistortion distortion = { 0 };
            OBD2CProfile       d2cProfile{};
            auto               vsp = sp->as<VideoStreamProfile>();

            auto &d2cProfileList = getD2CProfileList();
            if(!findBestMatchedD2CProfile(d2cProfileList, vsp, d2cProfile)) {
                throw libobsensor::unsupported_operation_exception("Can not find matched camera param!");
            }

            const auto &calibrationCameraParamList = getCalibrationCameraParamList();
            const auto &param                      = calibrationCameraParamList.at(d2cProfile.paramIndex);
            auto        postProcessParam           = d2cProfile.postProcessParam;

            auto streamType = sp->getType();
            if(streamType == OB_STREAM_COLOR) {
                intrinsic  = param.rgbIntrinsic;
                distortion = param.rgbDistortion;
            }
            else {
                intrinsic  = param.depthIntrinsic;
                distortion = param.depthDistortion;
            }

            if(mirrored) {
                // mirror intrinsic
                intrinsic.cx  = (float)1.0 * intrinsic.width - intrinsic.cx - 1;
                distortion.p2 = -distortion.p2;
                std::swap(postProcessParam.alignLeft, postProcessParam.alignRight);
            }

            if(streamType == OB_STREAM_COLOR) {
                intrinsic.fx = postProcessParam.depthScale * intrinsic.fx;
                intrinsic.fy = postProcessParam.depthScale * intrinsic.fy;
                intrinsic.cx = postProcessParam.depthScale * intrinsic.cx + postProcessParam.alignLeft;
                intrinsic.cy = postProcessParam.depthScale * intrinsic.cy + postProcessParam.alignTop;
                intrinsic.width =
                    static_cast<int16_t>(postProcessParam.depthScale * intrinsic.width + postProcessParam.alignLeft + postProcessParam.alignRight);
                intrinsic.height =
                    static_cast<int16_t>(postProcessParam.depthScale * intrinsic.height + postProcessParam.alignTop + postProcessParam.alignBottom);
            }

            auto ratio = (float)vsp->getWidth() / (float)intrinsic.width;
            intrinsic.fx *= ratio;
            intrinsic.fy *= ratio;
            intrinsic.cx *= ratio;
            intrinsic.cy *= ratio;
            intrinsic.width  = static_cast<int16_t>(vsp->getWidth());
            intrinsic.height = static_cast<int16_t>((float)intrinsic.height * ratio);

            intrinsicMgr->registerVideoStreamIntrinsics(sp, intrinsic);
            intrinsicMgr->registerVideoStreamDistortion(sp, distortion);
        }
    }
}

DisparityAlgParamManagerBase::DisparityAlgParamManagerBase(IDevice *device) : AlgParamManagerBase(device) {}

void DisparityAlgParamManagerBase::bindStreamProfileParams(std::vector<std::shared_ptr<const StreamProfile>> streamProfileList) {
    AlgParamManagerBase::bindStreamProfileParams(streamProfileList);
    bindDisparityParam(streamProfileList);
}

const OBDisparityParam &DisparityAlgParamManagerBase::getDisparityParam() const {
    return disparityParam_;
}

void DisparityAlgParamManagerBase::bindDisparityParam(std::vector<std::shared_ptr<const StreamProfile>> streamProfileList) {
    auto dispParam    = getDisparityParam();
    auto intrinsicMgr = StreamIntrinsicsManager::getInstance();
    for(const auto &sp: streamProfileList) {
        if(!sp->is<DisparityBasedStreamProfile>()) {
            continue;
        }
        intrinsicMgr->registerDisparityBasedStreamDisparityParam(sp, dispParam);
    }
}

}  // namespace libobsensor