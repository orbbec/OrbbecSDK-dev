#include "FrameIMUCorrectProcess.hpp"
#include "exception/ObException.hpp"
#include "logger/LoggerInterval.hpp"
#include "frame/FrameFactory.hpp"

namespace libobsensor {

IMUCorrecter::IMUCorrecter() : processFuncEnable_(false), uintTransformOnly_(false), correctionMode_(NORMAL_CORRECTION_MODE) {
    maxFilterFrameQueueSize_ = 100;
    srcFrameQueue_           = make_unique<single_consumer_frame_queue<std::shared_ptr<Frame>>>(maxFilterFrameQueueSize_);
}

std::shared_ptr<Frame> IMUCorrecter::processFunc(std::shared_ptr<const Frame> frame) {

    static constexpr float gravity = 9.80665f;
    static constexpr float deg2rad = 0.017453293f;
    if(frame->getType() == OB_FRAME_ACCEL) {
        OBAccelFrameData *frameData = (OBAccelFrameData *)frame->getData();
        if(uintTransformOnly_) {
            frameData->accelData[0] *= gravity;
            frameData->accelData[1] *= gravity;
            frameData->accelData[2] *= gravity;
        }
        else {
            float accelData[3] = { calculateAccelGravi(frameData->accelData[0], accelScaleRange_),
                                   calculateAccelGravi(frameData->accelData[1], accelScaleRange_),
                                   calculateAccelGravi(frameData->accelData[2], accelScaleRange_) };
            if(processFuncEnable_) {
                Eigen::Vector3d accelEigenVec(accelData[0], accelData[1], accelData[2]);
                accelEigenVec = correctAccel(accelEigenVec, &param_);
    
                accelData[0] = accelEigenVec.x();
                accelData[1] = accelEigenVec.y();
                accelData[2] = accelEigenVec.z();
            }

            memcpy(frameData->accelData, accelData, sizeof(float) * 3);
            frameData->temp = calculateRegisterTemperature(frameData->temp);
        }
        return frame;
    }
    else if(frame->getType() == OB_FRAME_GYRO) {
        // 适配IMU旧打包方案数据解析情况
        if(uintTransformOnly_) {
            OBGyroFrameData *gyroFrameData = (OBGyroFrameData *)frame->getData();
            gyroFrameData->gyroData[0] *= deg2rad;
            gyroFrameData->gyroData[1] *= deg2rad;
            gyroFrameData->gyroData[2] *= deg2rad;
        }
        return frame;
    }
    else if(frame->is<FrameSet>()) {
        auto             frameSet      = frame->as<FrameSet>();
        OBGyroFrameData *gyroFrameData = (OBGyroFrameData *)frameSet->getGyroFrame()->getData();
        float            gyroData[3]   = { calculateGyroDPS(gyroFrameData->gyroData[0], gyroScaleRange_),
                                           calculateGyroDPS(gyroFrameData->gyroData[1], gyroScaleRange_),
                                           calculateGyroDPS(gyroFrameData->gyroData[2], gyroScaleRange_) };

        OBAccelFrameData *accelFrameData = (OBAccelFrameData *)frameSet->getAccelFrame()->getData();
        float             accelData[3]   = { calculateAccelGravi(accelFrameData->accelData[0], accelScaleRange_),
                                             calculateAccelGravi(accelFrameData->accelData[1], accelScaleRange_),
                                             calculateAccelGravi(accelFrameData->accelData[2], accelScaleRange_) };

        if(processFuncEnable_) {
            Eigen::Vector3d gyroEigenVec(gyroData[0], gyroData[1], gyroData[2]);
            gyroEigenVec = correctGyro(gyroEigenVec, &param_);
            gyroData[0] = gyroEigenVec.x();
            gyroData[1] = gyroEigenVec.y();
            gyroData[2] = gyroEigenVec.z();
        }
        memcpy(gyroFrameData->gyroData, gyroData, sizeof(float) * 3);
        gyroFrameData->temp = calculateRegisterTemperature(frameData->temp);
        return frameSet->getGyroFrame();
    }

    return frame;
}
}  // namespace libobsensor
