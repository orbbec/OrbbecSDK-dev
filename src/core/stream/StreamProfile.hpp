#pragma once
 #include "IStreamProfile.hpp"
 #include "ISensor.hpp"
#include "openobsdk/h/ObTypes.h"
#include "exception/ObException.hpp"
#include <memory>
#include <vector>

namespace libobsensor {

class StreamProfile : public std::enable_shared_from_this<StreamProfile> {
public:
    StreamProfile(std::shared_ptr<LazySensor> owner, OBStreamType type, OBFormat format);

    virtual ~StreamProfile() noexcept = default;

   std::shared_ptr<LazySensor> getOwner() const;
    void                        bindOwner(std::shared_ptr<LazySensor> owner);
    void                        setType(OBStreamType type);
    OBStreamType                getType() const;
    void                        setFormat(OBFormat format);
    OBFormat                    getFormat() const;
    void                        setIndex(uint8_t index);
    uint8_t                     getIndex() const;

    OBExtrinsic getExtrinsicTo(std::shared_ptr<const StreamProfile> targetStreamProfile) const;
    void        bindExtrinsicTo(std::shared_ptr<const StreamProfile> targetStreamProfile, const OBExtrinsic &extrinsic);
    void        bindSameExtrinsicTo(std::shared_ptr<const StreamProfile> targetStreamProfile);

    virtual std::shared_ptr<StreamProfile> clone() const = 0;

    template <typename T> bool               is() const;
    template <typename T> std::shared_ptr<T> as() {
        if(!is<T>())
            throw unsupported_operation_exception("unsupported operation, object's type is not require type");

        return std::dynamic_pointer_cast<T>(shared_from_this());
    }

    template <typename T> std::shared_ptr<const T> as() const {
        if(!is<T>())
            throw unsupported_operation_exception("unsupported operation, object's type is not require type");

        return std::dynamic_pointer_cast<const T>(shared_from_this());
    }

protected:
    std::weak_ptr<LazySensor> owner_;
    OBStreamType           type_;
    OBFormat               format_;
    uint8_t                index_;  // for multi-stream sensor (multi pin uvc device)
};

struct StreamProfileWeakPtrCompare {
    bool operator()(const std::weak_ptr<const StreamProfile> &a, const std::weak_ptr<const StreamProfile> &b) const {
        auto sharedA = a.lock();
        auto sharedB = b.lock();

        if(sharedA && sharedB) {
            return sharedA < sharedB;
        }

        return sharedA != nullptr;
    }
};

class VideoStreamProfile : public StreamProfile {
public:
    VideoStreamProfile(std::shared_ptr<LazySensor> owner, OBStreamType type, OBFormat format, uint32_t width, uint32_t height, uint32_t fps);

    VideoStreamProfile(std::shared_ptr<const VideoStreamProfile> other) = delete;

    bool operator==(const VideoStreamProfile &other) const;

    ~VideoStreamProfile() noexcept override = default;

    void               setWidth(uint32_t width);
    uint32_t           getWidth() const;
    void               setHeight(uint32_t height);
    uint32_t           getHeight() const;
    uint32_t           getFps() const;
    OBCameraIntrinsic  getIntrinsic() const;
    void               bindIntrinsic(const OBCameraIntrinsic &intrinsic);
    OBCameraDistortion getDistortion() const;
    void               bindDistortion(const OBCameraDistortion &distortion);

    uint32_t getMaxFrameDataSize() const;

    std::shared_ptr<StreamProfile> clone() const override;

protected:
    uint32_t width_;
    uint32_t height_;
    uint32_t fps_;
};

class DisparityStreamProfile : public VideoStreamProfile {
public:
    DisparityStreamProfile(std::shared_ptr<LazySensor> owner, OBStreamType type, OBFormat format, uint32_t width, uint32_t height, uint32_t fps);
    ~DisparityStreamProfile() noexcept override = default;

    OBDisparityProcessParam getDisparityProcessParam() const;
    void                    bindDisparityProcessParam(const OBDisparityProcessParam &param);
};

class AccelStreamProfile : public StreamProfile {
public:
    AccelStreamProfile(std::shared_ptr<LazySensor> owner, OBAccelFullScaleRange fullScaleRange, OBAccelSampleRate sampleRate);

    ~AccelStreamProfile() noexcept override = default;

    OBAccelFullScaleRange          getFullScaleRange() const;
    OBAccelSampleRate              getSampleRate() const;
    void                           bindIntrinsic(const OBAccelIntrinsic &intrinsic);
    OBAccelIntrinsic               getIntrinsic() const;
    std::shared_ptr<StreamProfile> clone() const override;

protected:
    OBAccelFullScaleRange fullScaleRange_;
    OBAccelSampleRate     sampleRate_;
};

class GyroStreamProfile : public StreamProfile {
public:
    GyroStreamProfile(std::shared_ptr<LazySensor> owner, OBGyroFullScaleRange fullScaleRange, OBGyroSampleRate sampleRate);

    ~GyroStreamProfile() noexcept override = default;

    OBGyroFullScaleRange           getFullScaleRange() const;
    OBGyroSampleRate               getSampleRate() const;
    void                           bindIntrinsic(const OBGyroIntrinsic &intrinsic);
    OBGyroIntrinsic                getIntrinsic() const;
    std::shared_ptr<StreamProfile> clone() const override;

protected:
    OBGyroFullScaleRange fullScaleRange_;
    OBGyroSampleRate     sampleRate_;
};

template <typename T> bool StreamProfile::is() const {
    switch(type_) {
    case OB_STREAM_VIDEO:
    case OB_STREAM_IR:
    case OB_STREAM_IR_LEFT:
    case OB_STREAM_IR_RIGHT:
    case OB_STREAM_COLOR:
    case OB_STREAM_DEPTH:
    case OB_STREAM_RAW_PHASE:
        return typeid(T) == typeid(VideoStreamProfile);
    case OB_STREAM_ACCEL:
        return typeid(T) == typeid(AccelStreamProfile);
    case OB_STREAM_GYRO:
        return typeid(T) == typeid(GyroStreamProfile);
    case OB_STREAM_DISPARITY:
        return typeid(T) == typeid(DisparityStreamProfile);
    default:
        break;
    }
    return false;
}

std::vector<std::shared_ptr<const VideoStreamProfile>> matchVideoStreamProfile(const StreamProfileList &profileList, uint32_t width, uint32_t height,
                                                                               uint32_t fps, OBFormat format);

std::vector<std::shared_ptr<const AccelStreamProfile>> matchAccelStreamProfile(const StreamProfileList &profileList, OBAccelFullScaleRange fullScaleRange,
                                                                               OBAccelSampleRate sampleRate);

std::vector<std::shared_ptr<const GyroStreamProfile>> matchGyroStreamProfile(const StreamProfileList &profileList, OBGyroFullScaleRange fullScaleRange,
                                                                             OBGyroSampleRate sampleRate);

}  // namespace libobsensor

