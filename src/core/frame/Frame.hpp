// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec Corporation. All Rights Reserved.

#pragma once

#include "IFrameMetadataParser.hpp"
#include "IStreamProfile.hpp"
#include "exception/ObException.hpp"
#include "IFrame.hpp"

#include <atomic>
#include <memory>
#include <mutex>
#include <vector>
#include <typeinfo>

namespace libobsensor {

class FrameSet;
class PointsFrame;
class VideoFrame;
class ColorFrame;
class DepthFrame;
class IRFrame;
class AccelFrame;
class GyroFrame;

using FrameBufferReclaimFunc = std::function<void(void)>;

class Frame : public std::enable_shared_from_this<Frame> {
public:
    Frame(uint8_t *data, size_t dataBufSize, OBFrameType type, FrameBufferReclaimFunc bufferReclaimFunc = nullptr);
    virtual ~Frame() noexcept;

    OBFrameType getType() const;

    uint64_t       getNumber() const;
    void           setNumber(const uint64_t number);
    size_t         getDataSize() const;
    const uint8_t *getData() const;
    uint8_t       *getDataUnsafe() const;  // use with caution, data may be changed while other threads are using it
    void           updateData(const uint8_t *data, size_t dataSize);
    uint64_t       getTimeStampUsec() const;
    void           setTimeStampUsec(uint64_t ts);
    uint64_t       getSystemTimeStampUsec() const;
    void           setSystemTimeStampUsec(uint64_t ts);
    uint64_t       getGlobalTimeStampUsec() const;
    void           setGlobalTimeStampUsec(uint64_t ts);

    size_t         getMetadataSize() const;
    void           updateMetadata(const uint8_t *metadata, size_t metadataSize);
    const uint8_t *getMetadata() const;

    void    registerMetadataParsers(std::shared_ptr<IFrameMetadataParserContainer> parsers);
    bool    hasMetadata(OBFrameMetadataType type) const;
    int64_t getMetadataValue(OBFrameMetadataType type) const;

    std::shared_ptr<const StreamProfile> getStreamProfile() const;
    void                                 setStreamProfile(std::shared_ptr<const StreamProfile> streamProfile);
    OBFormat                             getFormat() const;  // get from stream profile

    virtual void copyInfo(std::shared_ptr<const Frame> sourceFrame);

    template <typename T> bool               is() const;
    template <typename T> std::shared_ptr<T> as() {
        if(!is<T>()) {
            throw unsupported_operation_exception("unsupported operation, object's type is not require type");
        }

        return std::dynamic_pointer_cast<T>(shared_from_this());
    }

    template <typename T> std::shared_ptr<const T> as() const {
        if(!is<T>())
            throw unsupported_operation_exception("unsupported operation, object's type is not require type");

        return std::dynamic_pointer_cast<const T>(shared_from_this());
    }

protected:
    size_t getDataBufSize() const;

protected:
    size_t                                         dataSize_;
    uint64_t                                       number_;
    uint64_t                                       timeStampUsec_;
    uint64_t                                       systemTimeStampUsec_;
    uint64_t                                       globalTimeStampUsec_;
    size_t                                         metadataSize_;
    uint8_t                                        metadata_[256];
    std::shared_ptr<IFrameMetadataParserContainer> metadataPhasers_;
    std::shared_ptr<const StreamProfile>           streamProfile_;

    const OBFrameType type_;  // Determined during construction, it is an inherent property of the object and cannot be changed.

private:
    uint8_t const         *frameData_;
    const size_t           dataBufSize_;
    FrameBufferReclaimFunc bufferReclaimFunc_;
};

class VideoFrame : public Frame {
public:
    VideoFrame(uint8_t *data, size_t dataBufSize, OBFrameType type, FrameBufferReclaimFunc bufferReclaimFunc = nullptr);

    uint32_t getWidth() const;
    uint32_t getHeight() const;
    uint32_t getFps() const;

    void     setStride(uint32_t stride);
    uint32_t getStride() const;

    uint8_t getPixelAvailableBitSize() const;
    void    setPixelAvailableBitSize(uint8_t bitSize);

    virtual void copyInfo(std::shared_ptr<const Frame> sourceFrame) override;

protected:
    uint8_t  availableBitSize_;  // available bit size of each pixel
    uint32_t stride_ = 0;
};

class ColorFrame : public VideoFrame {
public:
    ColorFrame(uint8_t *data, size_t dataBufSize, FrameBufferReclaimFunc bufferReclaimFunc = nullptr);
};

class DepthFrame : public VideoFrame {
public:
    DepthFrame(uint8_t *data, size_t dataBufSize, FrameBufferReclaimFunc bufferReclaimFunc = nullptr);

    void  setValueScale(float valueScale);
    float getValueScale() const;

    virtual void copyInfo(std::shared_ptr<const Frame> sourceFrame) override;

private:
    float valueScale_;
};

class IRFrame : public VideoFrame {
public:
    IRFrame(uint8_t *data, size_t dataBufSize, FrameBufferReclaimFunc bufferReclaimFunc = nullptr, OBFrameType frameType = OB_FRAME_IR);
};

class IRLeftFrame : public IRFrame {
public:
    IRLeftFrame(uint8_t *data, size_t dataBufSize, FrameBufferReclaimFunc bufferReclaimFunc = nullptr);
};

class IRRightFrame : public IRFrame {
public:
    IRRightFrame(uint8_t *data, size_t dataBufSize, FrameBufferReclaimFunc bufferReclaimFunc = nullptr);
};

class PointsFrame : public Frame {
public:
    PointsFrame(uint8_t *data, size_t dataBufSize, FrameBufferReclaimFunc bufferReclaimFunc = nullptr);

    void  setCoordinateValueScale(float valueScale);
    float getCoordinateValueScale() const;

private:
    float coordValueScale_;  // coordinate value scale, multiply by this value to get actual coordinate value in mm
};

class AccelFrame : public Frame {
public:
    typedef struct {
        float accelData[3];  // Acceleration values in three directions (xyz), unit: g (9.80665 m/s^2)
        float temp;          // Temperature in Celsius
    } OBAccelFrameData;

public:
    AccelFrame(uint8_t *data, size_t dataBufSize, FrameBufferReclaimFunc bufferReclaimFunc = nullptr);

    OBAccelValue value();
    float        temperature();
};

class GyroFrame : public Frame {
public:
    typedef struct {
        float gyroData[3];  // Acceleration values ​​in three directions (xyz), unit: dps (degrees per second)
        float temp;         // Temperature in Celsius
    } OBGyroFrameData;

public:
    GyroFrame(uint8_t *data, size_t dataBufSize, FrameBufferReclaimFunc bufferReclaimFunc = nullptr);

    OBGyroValue value();
    float       temperature();
};

class FrameSet : public Frame {
    typedef std::function<bool(void *)> ForeachBack;

public:
    FrameSet(uint8_t *data, size_t dataBufSize, FrameBufferReclaimFunc bufferReclaimFunc = nullptr);
    ~FrameSet() noexcept;

    uint32_t getFrameCount();

    std::shared_ptr<Frame> getDepthFrame();
    std::shared_ptr<Frame> getIRFrame();
    std::shared_ptr<Frame> getColorFrame();
    std::shared_ptr<Frame> getAccelFrame();
    std::shared_ptr<Frame> getGyroFrame();
    std::shared_ptr<Frame> getPointsFrame();
    std::shared_ptr<Frame> getFrame(OBFrameType frameType);
    std::shared_ptr<Frame> getFrame(int index);

    // It is recommended to use the rvalue reference interface. If you really need it, you can uncomment the following
    // void pushFrame(std::shared_ptr<Frame> frame);
    void pushFrame(std::shared_ptr<Frame> &&frame);
    void clearAllFrame();

public:
    void foreachFrame(ForeachBack foreachBack);
};

template <typename T> bool Frame::is() const {
    switch(type_) {

    case OB_FRAME_VIDEO:
        return (typeid(T) == typeid(IRFrame) || typeid(T) == typeid(DepthFrame) || typeid(T) == typeid(ColorFrame) || typeid(T) == typeid(VideoFrame));
    case OB_FRAME_IR:
        return (typeid(T) == typeid(IRFrame) || typeid(T) == typeid(VideoFrame));
    case OB_FRAME_IR_LEFT:
        return (typeid(T) == typeid(IRLeftFrame) || typeid(T) == typeid(VideoFrame));
    case OB_FRAME_IR_RIGHT:
        return (typeid(T) == typeid(IRRightFrame) || typeid(T) == typeid(VideoFrame));
    case OB_FRAME_DEPTH:
        return (typeid(T) == typeid(DepthFrame) || typeid(T) == typeid(VideoFrame));
    case OB_FRAME_COLOR:
        return (typeid(T) == typeid(ColorFrame) || typeid(T) == typeid(VideoFrame));
    case OB_FRAME_ACCEL:
        return (typeid(T) == typeid(AccelFrame));
    case OB_FRAME_GYRO:
        return (typeid(T) == typeid(GyroFrame));
    case OB_FRAME_SET:
        return (typeid(T) == typeid(FrameSet));
    case OB_FRAME_POINTS:
        return (typeid(T) == typeid(PointsFrame));
    default:
        break;
    }
    return false;
}

}  // namespace libobsensor
