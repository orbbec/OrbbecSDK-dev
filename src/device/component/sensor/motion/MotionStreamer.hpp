#pragma once

#include "IFilter.hpp"
#include "ISourcePort.hpp"

#include <atomic>
#include <map>
#include <mutex>

namespace libobsensor{

typedef struct {
    float accelData[3];  // 三个方向加速度值(xyz)，单位：g
    float temp;          // 摄氏度
} OBAccelFrameData;

typedef struct {
    float gyroData[3];  // 三个方向加速度值(xyz)，单位：dps
    float temp;         // 摄氏度
} OBGyroFrameData;

// 原始IMU数据，软件打包方式，需要在SDK端进行计算
typedef struct {
    uint8_t  reportId;    // 固件固定传1
    uint8_t  sampleRate;  // OB_SAMPLE_RATE
    uint8_t  groupLen;    // sizeof(OBImuOriginData)
    uint8_t  groupCount;  // 一包内有多少帧数据。
    uint32_t reserved;    // 保留
} OBImuHeader;

typedef struct {
    int16_t  groupId;  // 一包内的第几组
    int16_t  accelX;
    int16_t  accelY;
    int16_t  accelZ;
    int16_t  gyroX;
    int16_t  gyroY;
    int16_t  gyroZ;
    int16_t  temperature;
    uint32_t timestamp[2];
} OBImuOriginData;

class MotionStreamer final {
public:
    MotionStreamer(const std::shared_ptr<IDataStreamPort> &backend, const std::shared_ptr<IFilter>& dataPhaser);

    ~MotionStreamer() noexcept;

    void start(std::shared_ptr<const StreamProfile> sp, FrameCallback callback);
    void stop(std::shared_ptr<const StreamProfile> sp);

private:
    void praseIMUData(std::shared_ptr<Frame> frame);

private:
    std::shared_ptr<IDataStreamPort> backend_;
    std::shared_ptr<IFilter> dataPhaser_;

    std::mutex mtx_;
    std::map<std::shared_ptr<const StreamProfile>, FrameCallback> callbacks_;

    std::atomic_bool running_;
};

}