#include "MotionStreamer.hpp"
#include "frame/Frame.hpp"
#include "stream/StreamProfile.hpp"
#include "logger/LoggerInterval.hpp"
#include "frame/FrameFactory.hpp"
#include "shared/utils/Utils.hpp"
#include "filter/public_filters/FrameIMUCorrectProcess.hpp"

namespace libobsensor {
MotionStreamer::MotionStreamer(const std::shared_ptr<IDataStreamPort> &backend, const std::shared_ptr<IFilter> &dataPhaser)
    : backend_(backend), dataPhaser_(dataPhaser), running_(false), frameIndex_(0) {
    dataPhaser_->setCallback([this](std::shared_ptr<const Frame> frame) {
        if(!frame) {
            return;
        }

        std::lock_guard<std::mutex> lock(cbMtx_);
        for(auto &callback: callbacks_) {
            auto callbackFrame = frame;
            auto format        = frame->getFormat();
            if(frame->is<FrameSet>()) {
                auto frameSet = frame->as<FrameSet>();
                callbackFrame = frameSet->getFrame(utils::mapStreamTypeToFrameType(callback.first->getType()));
                if(callbackFrame) {
                    format = callbackFrame->getFormat();
                }
            }

            if(format != callback.first->getFormat()) {
                continue;
            }
            callback.second(callbackFrame);
        }
    });
}

MotionStreamer::~MotionStreamer() noexcept {
    {
        std::lock_guard<std::mutex> lock(cbMtx_);
        callbacks_.clear();
    }

    if(running_) {
        auto dataStreamPort = std::dynamic_pointer_cast<IDataStreamPort>(backend_);
        dataStreamPort->stopStream();
        dataPhaser_->reset();
        running_ = false;
    }
}

void MotionStreamer::start(std::shared_ptr<const StreamProfile> sp, FrameCallback callback) {
    {
        std::lock_guard<std::mutex> lock(cbMtx_);
        callbacks_[sp] = callback;
        if(running_) {
            return;
        }
    }
    running_ = true;

    std::function<void(std::shared_ptr<Frame> frame)> streamCallback = std::bind(&MotionStreamer::praseIMUData, this, std::placeholders::_1);
    backend_->startStream(streamCallback);
}

void MotionStreamer::stop(std::shared_ptr<const StreamProfile> sp) {
    {
        std::lock_guard<std::mutex> lock(cbMtx_);
        auto                        iter = callbacks_.find(sp);
        if(iter == callbacks_.end()) {
            throw invalid_value_exception("Stop stream failed, stream profile not found.");
        }

        callbacks_.erase(iter);
        if(!callbacks_.empty()) {
            return;
        }
    }

    backend_->stopStream();
    dataPhaser_->reset();
    running_ = false;
}

void MotionStreamer::praseIMUData(std::shared_ptr<Frame> frame) {
    auto         data     = frame->getData();
    OBImuHeader *header   = (OBImuHeader *)data;
    auto         dataSize = frame->getDataSize();
    if(header->reportId != 1) {
        LOG_WARN_INTVL("Imu header is invalid,drop imu package!");
        return;
    }
    const auto computeDataSize = sizeof(OBImuHeader) + sizeof(OBImuOriginData) * header->groupCount;
    if(dataSize < computeDataSize) {
        LOG_WARN_INTVL("Imu header is invalid,drop imu package!, invalid data size. dataSize={}, computeDataSize={}, groupCount={}", dataSize, computeDataSize,
                       header->groupCount);
        return;
    }
    const auto computeDataSizeP = sizeof(OBImuHeader) + header->groupLen * header->groupCount;
    if(dataSize < computeDataSizeP) {
        LOG_WARN_INTVL("Imu header is invalid,drop imu package!, invalid data size. dataSize={}, computeDataSizeP={}, groupCount={}", dataSize,
                       computeDataSizeP, header->groupCount);
        return;
    }

    int                                       offset     = sizeof(OBImuOriginData);
    uint8_t                                  *imuOrgData = (uint8_t *)data + sizeof(OBImuHeader);
    std::shared_ptr<const AccelStreamProfile> accelStreamProfile;
    std::shared_ptr<const GyroStreamProfile>  gyroStreamProfile;
    for(const auto &iter: callbacks_) {
        if(iter.first->is<libobsensor::AccelStreamProfile>()) {
            accelStreamProfile = iter.first->as<libobsensor::AccelStreamProfile>();
        }
        else if(iter.first->is<libobsensor::GyroStreamProfile>()) {
            gyroStreamProfile = iter.first->as<libobsensor::GyroStreamProfile>();
        }
    }

    for(int groupIndex = 0; groupIndex < header->groupCount; groupIndex++) {
        auto frameSet = FrameFactory::createFrameSet();
        if(frameSet == nullptr) {
            LOG_WARN_INTVL("acquire frame set failed, drop imu package!");
            return;
        }
        auto nowTimeUs = utils::getNowTimesUs();

        std::shared_ptr<Frame> accelFrame, gyroFrame;
        if(accelStreamProfile) {
            accelFrame = FrameFactory::createFrameFromStreamProfile(accelStreamProfile);
        }

        if(gyroStreamProfile) {
            gyroFrame = FrameFactory::createFrameFromStreamProfile(gyroStreamProfile);
        }

        OBImuOriginData *imuData = (OBImuOriginData *)((uint8_t *)imuOrgData + groupIndex * offset);

        if(accelFrame && accelStreamProfile && accelStreamProfile->getFullScaleRange() != OB_ACCEL_FS_UNKNOWN) {
            OBAccelFrameData *accelFrameData = (OBAccelFrameData *)accelFrame->getData();
            float             accelData[3]   = {
                IMUCorrecter::calculateAccelGravity(static_cast<int16_t>(imuData->accelX), static_cast<uint8_t>(accelStreamProfile->getFullScaleRange())),
                IMUCorrecter::calculateAccelGravity(static_cast<int16_t>(imuData->accelY), static_cast<uint8_t>(accelStreamProfile->getFullScaleRange())),
                IMUCorrecter::calculateAccelGravity(static_cast<int16_t>(imuData->accelZ), static_cast<uint8_t>(accelStreamProfile->getFullScaleRange()))
            };
            memcpy(accelFrameData->accelData, accelData, sizeof(float) * 3);
            accelFrameData->temp = imuData->temperature;
        }

        if(gyroFrame && gyroStreamProfile && gyroStreamProfile->getFullScaleRange() != OB_GYRO_FS_UNKNOWN) {
            OBGyroFrameData *gyroFrameData = (OBGyroFrameData *)gyroFrame->getData();
            float            gyroData[3]   = {
                IMUCorrecter::calculateGyroDPS(static_cast<int16_t>(imuData->gyroX), static_cast<uint8_t>(gyroStreamProfile->getFullScaleRange())),
                IMUCorrecter::calculateGyroDPS(static_cast<int16_t>(imuData->gyroY), static_cast<uint8_t>(gyroStreamProfile->getFullScaleRange())),
                IMUCorrecter::calculateGyroDPS(static_cast<int16_t>(imuData->gyroZ), static_cast<uint8_t>(gyroStreamProfile->getFullScaleRange()))
            };
            memcpy(gyroFrameData->gyroData, gyroData, sizeof(float) * 3);
            gyroFrameData->temp = imuData->temperature;
        }

        uint64_t timestamp = ((uint64_t)imuData->timestamp[0] | ((uint64_t)imuData->timestamp[1] << 32));
        if(accelFrame) {
            accelFrame->setNumber(frameIndex_++);
            accelFrame->setTimeStampUsec(timestamp);
            accelFrame->setSystemTimeStampUsec(nowTimeUs);
            frameSet->pushFrame(accelFrame);
        }

        if(gyroFrame) {
            gyroFrame->setNumber(frameIndex_++);
            gyroFrame->setTimeStampUsec(timestamp);
            gyroFrame->setSystemTimeStampUsec(nowTimeUs);
            frameSet->pushFrame(gyroFrame);
        }

        dataPhaser_->pushFrame(frameSet);
    }
}

}  // namespace libobsensor