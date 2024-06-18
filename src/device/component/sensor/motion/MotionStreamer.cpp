#include "MotionStreamer.hpp"
#include "frame/Frame.hpp"
#include "stream/StreamProfile.hpp"
#include "logger/LoggerInterval.hpp"
#include "frame/FrameFactory.hpp"
#include "shared/utils/Utils.hpp"
#include "filter/public_filters/FrameIMUCorrectProcess.hpp"

namespace libobsensor {
MotionStreamer::MotionStreamer(const std::shared_ptr<IDataStreamPort> &backend, const std::shared_ptr<IFilter> &dataPhaser)
    : backend_(backend), dataPhaser_(dataPhaser), running_(false) {
    dataPhaser_->setCallback([this](std::shared_ptr<Frame> frame) {
        std::lock_guard<std::mutex> lock(mtx_);
        auto                        format = frame->getFormat();
        for(auto &callback: callbacks_) {
            if(format != callback.first->getFormat()) {
                continue;
            }
            callback.second(frame);
        }
    });
}

MotionStreamer::~MotionStreamer() noexcept {
    std::lock_guard<std::mutex> lock(mtx_);
    callbacks_.clear();

    if(running_) {
        auto dataStreamPort = std::dynamic_pointer_cast<IDataStreamPort>(backend_);
        dataStreamPort->stopStream();
        dataPhaser_->reset();
        running_ = false;
    }
}

void MotionStreamer::start(std::shared_ptr<const StreamProfile> sp, FrameCallback callback) {
    std::lock_guard<std::mutex> lock(mtx_);
    callbacks_[sp] = callback;
    if(running_) {
        return;
    }

    running_ = true;

    std::function<void(std::shared_ptr<Frame> frame)> streamCallback = std::bind(&MotionStreamer::praseIMUData, this, std::placeholders::_1);
    backend_->startStream(streamCallback);
}

void MotionStreamer::stop(std::shared_ptr<const StreamProfile> sp) {
    std::lock_guard<std::mutex> lock(mtx_);
    auto                        iter = callbacks_.find(sp);
    if(iter == callbacks_.end()) {
        throw invalid_value_exception("Stop stream failed, stream profile not found.");
    }

    callbacks_.erase(iter);
    if(!callbacks_.empty()) {
        return;
    }

    backend_->stopStream();
    dataPhaser_->reset();
    running_ = false;
}

void MotionStreamer::praseIMUData(std::shared_ptr<Frame> frame){
    auto data = frame->getData();
    OBImuHeader *header = (OBImuHeader *)data;
    auto         dataSize = frame->getDataSize();
    if(header->reportId != 1) {
        LOG_WARN_INTVL("Imu header is invalid,drop imu package!");
        return;
    }
    const auto computeDataSize = sizeof(OBImuHeader) + sizeof(OBImuOriginData) * header->groupCount;
    if(dataSize < computeDataSize) {
        LOG_WARN_INTVL("Imu header is invalid,drop imu package!, invalid data size. dataSize={}, computeDataSize={}, groupCount={}", dataSize,
                       computeDataSize, header->groupCount);
        return;
    }
    const auto computeDataSizeP = sizeof(OBImuHeader) + header->groupLen * header->groupCount;
    if(dataSize < computeDataSizeP) {
        LOG_WARN_INTVL("Imu header is invalid,drop imu package!, invalid data size. dataSize={}, computeDataSizeP={}, groupCount={}", dataSize,
                       computeDataSizeP, header->groupCount);
        return;
    }

    int      offset     = sizeof(OBImuOriginData);
    uint8_t *imuOrgData = (uint8_t *)data + sizeof(OBImuHeader);
    OBAccelFullScaleRange accelFullScaleRange = OB_ACCEL_FS_UNKNOW;
    OBGyroFullScaleRange  gyroFullScaleRange  = OB_GYRO_FS_UNKNOW;
    for(const auto &iter :callbacks_){
        if(iter.first->is<libobsensor::AccelStreamProfile>()){
            accelFullScaleRange = iter.first->as<libobsensor::AccelStreamProfile>()->getFullScaleRange();
        }else if (iter.first->is<libobsensor::GyroStreamProfile>()){
            gyroFullScaleRange = iter.first->as<libobsensor::GyroStreamProfile>()->getFullScaleRange();            
        }
    }

    for(int groupIndex = 0; groupIndex < header->groupCount; groupIndex++) {
        auto frameSet = FrameFactory::createFrameSet();
        if(frameSet == nullptr) {
            LOG_WARN_INTVL("acquire frame set failed, drop imu package!");
            return;
        }
        auto accelFrame = std::const_pointer_cast<Frame>(frameSet->getFrame(OB_FRAME_ACCEL));
        auto gyroFrame = std::const_pointer_cast<Frame>(frameSet->getFrame(OB_FRAME_GYRO));
        auto nowTimeUs = utils::getNowTimesUs();
        OBImuOriginData *imuData = (OBImuOriginData *)((uint8_t *)imuOrgData + groupIndex * offset);

        uint64_t timestamp = ((uint64_t)imuData->timestamp[0] | ((uint64_t)imuData->timestamp[1] << 32));
        // accelFrame->setNumber((uint32_t)frameIndex_++);
        accelFrame->setTimeStampUsec(timestamp);
        gyroFrame->setTimeStampUsec(timestamp);
        // backend_.frameTimestampConverter->convert(timestamp, frame);
        accelFrame->setSystemTimeStampUsec(nowTimeUs);
        gyroFrame->setSystemTimeStampUsec(nowTimeUs);

        if(accelFullScaleRange != OB_ACCEL_FS_UNKNOW){
            OBAccelFrameData *accelFrameData    = (OBAccelFrameData *)accelFrame->getData();
            float             accelData[3] = { 
                IMUCorrecter::calculateAccelGravity(static_cast<int16_t>(imuData->accelX),static_cast<uint8_t>(accelFullScaleRange)), 
                IMUCorrecter::calculateAccelGravity(static_cast<int16_t>(imuData->accelY),static_cast<uint8_t>(accelFullScaleRange)), 
                IMUCorrecter::calculateAccelGravity(static_cast<int16_t>(imuData->accelZ),static_cast<uint8_t>(accelFullScaleRange)) };
            memcpy(accelFrameData->accelData, accelData, sizeof(float) * 3);
            accelFrameData->temp = imuData->temperature;
        }

        if(gyroFullScaleRange != OB_GYRO_FS_UNKNOW){    
            OBGyroFrameData *gyroFrameData = (OBGyroFrameData *)gyroFrame->getData();
            float             gyroData[3] = { 
                IMUCorrecter::calculateGyroDPS(static_cast<int16_t>(imuData->gyroX),static_cast<uint8_t>(gyroFullScaleRange)), 
                IMUCorrecter::calculateGyroDPS(static_cast<int16_t>(imuData->gyroY),static_cast<uint8_t>(gyroFullScaleRange)), 
                IMUCorrecter::calculateGyroDPS(static_cast<int16_t>(imuData->gyroZ),static_cast<uint8_t>(gyroFullScaleRange)) };
            memcpy(gyroFrameData->gyroData, gyroData, sizeof(float) * 3);
            gyroFrameData->temp = imuData->temperature;
        }

        dataPhaser_->pushFrame(frameSet);
    }
}

}  // namespace libobsensor