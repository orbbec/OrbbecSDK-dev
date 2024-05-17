// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec Corporation. All Rights Reserved.

#pragma once

#include "IFrameMetadataParser.hpp"
#include "stream/StreamProfile.hpp"
#include "exception/OBException.hpp"

#include <atomic>
#include <memory>
#include <mutex>
#include <vector>
#include <typeinfo>

namespace ob {
namespace core {

class FrameSet;
class PointsFrame;
class VideoFrame;
class ColorFrame;
class DepthFrame;
class IRFrame;
class AccelFrame;
class GyroFrame;

using FrameBufferReclaim = std::function<void(void)>;

class Frame : public std::enable_shared_from_this<Frame> {
public:
    Frame(uint8_t *data, uint32_t dataBufSize, OBFrameType type, FrameBufferReclaim customBufferReclaim = nullptr);
    virtual ~Frame() noexcept;

    virtual OBFrameType getType();

    virtual OBFormat       getFormat() const;
    virtual void           setFormat(const OBFormat format);
    virtual uint32_t       getFps() const;
    virtual void           setFps(const uint32_t fps);
    virtual uint32_t       getNumber() const;
    virtual void           setNumber(const uint32_t number);
    virtual uint32_t       getDataSize() const;
    virtual void           setDataSize(uint32_t dataSize);
    virtual const uint8_t *getData() const;
    virtual void           updateData(const uint8_t *data, uint32_t dataSize);
    virtual double         getTimeStampMsec() const;
    virtual void           setTimeStampMsec(double ts);
    virtual double         getSystemTimeStampMsec() const;
    virtual void           setSystemTimeStampMsec(double ts);
    virtual double         getGlobalTimeStampMsec() const;
    virtual void           setGlobalTimeStampMsec(double ts);
    virtual uint32_t       getWidth() const;
    virtual void           setWidth(uint32_t width);
    virtual uint32_t       getHeight() const;
    virtual void           setHeight(uint32_t height);
    virtual uint32_t       getStride() const;
    virtual void           setStride(uint32_t stride);
    virtual uint32_t       getBytesPerPixel() const;

    virtual uint32_t       getMetadataSize() const;
    virtual void           updateMetadata(const uint8_t *metadata, uint32_t metadataSize);
    virtual const uint8_t *getMetadata() const;

    virtual void    registerMetadataParsers(std::shared_ptr<IFrameMetadataParserContainer> parsers);
    virtual bool    hasMetadata(OBFrameMetadataType type) const;
    virtual int64_t getMetadataValue(OBFrameMetadataType type) const;

    virtual std::shared_ptr<const StreamProfile> getStreamProfile() const;
    virtual void                                 setStreamProfile(std::shared_ptr<const StreamProfile> streamProfile);

    virtual void copyInfo(std::shared_ptr<Frame> sourceFrame);

    template <typename T> bool               is();
    template <typename T> std::shared_ptr<T> as() {
        if(!is<T>()) {
            throw unsupported_operation_exception("unsupported operation, object's type is not require type");
        }

        return std::static_pointer_cast<T>(std::const_pointer_cast<Frame>(shared_from_this()));
    }

protected:
    uint32_t getDataBufSize() const;

protected:
    uint32_t                                       dataSize_            = 0;
    uint32_t                                       fps_                 = 0;
    uint32_t                                       number_              = 0;
    double                                         timeStampMsec_       = 0;
    double                                         systemTimeStampMsec_ = 0;
    double                                         globalTimeStampMsec_ = 0;
    OBFormat                                       format_              = OB_FORMAT_UNKNOWN;
    uint32_t                                       width_               = 0;
    uint32_t                                       height_              = 0;
    uint32_t                                       stride_              = 0;
    uint32_t                                       metadataSize_        = 0;
    uint8_t                                        metadata_[256];
    std::shared_ptr<IFrameMetadataParserContainer> metadataPhasers_ = nullptr;
    std::shared_ptr<const StreamProfile>           streamProfile_   = nullptr;

    const OBFrameType type_;  // Determined during construction, it is an inherent property of the object and cannot be changed.

private:
    uint8_t const     *frameData_;
    const uint32_t     dataBufSize_;
    FrameBufferReclaim customBufferReclaim_;
};

class VideoFrame : public Frame {
public:
    VideoFrame(uint8_t *data, uint32_t dataBufSize, OBFrameType type, FrameBufferReclaim customBufferReclaim = nullptr);

    virtual void setFormat(const OBFormat format) override;

    virtual uint8_t  getScrDataSize() const;
    virtual void     updateScrData(const uint8_t *scrData, uint8_t scrDataSize);
    virtual uint8_t *getScrData();

    virtual uint8_t getPixelAvailableBitSize() const;
    virtual void    setPixelAvailableBitSize(uint8_t bitSize);

    virtual void copyInfo(std::shared_ptr<Frame> sourceFrame) override;

protected:
    uint8_t availableBitSize_;

    uint8_t scrDataSize_;
    uint8_t scrData_[12];
};

class ColorFrame : public VideoFrame {
public:
    ColorFrame(uint8_t *data, uint32_t dataBufSize, FrameBufferReclaim customBufferReclaim = nullptr);
};

class DepthFrame : public VideoFrame {
public:
    DepthFrame(uint8_t *data, uint32_t dataBufSize, FrameBufferReclaim customBufferReclaim = nullptr);

    void  setValueScale(float valueScale);
    float getValueScale() const;

    virtual void copyInfo(std::shared_ptr<Frame> sourceFrame) override;

private:
    float valueScale_;
};

class IRFrame : public VideoFrame {
public:
    IRFrame(uint8_t *data, uint32_t dataBufSize, FrameBufferReclaim customBufferReclaim = nullptr, OBFrameType frameType = OB_FRAME_IR);
};

class IRLeftFrame : public IRFrame {
public:
    IRLeftFrame(uint8_t *data, uint32_t dataBufSize, FrameBufferReclaim customBufferReclaim = nullptr);
};

class IRRightFrame : public IRFrame {
public:
    IRRightFrame(uint8_t *data, uint32_t dataBufSize, FrameBufferReclaim customBufferReclaim = nullptr);
};

class PointsFrame : public Frame {
public:
    PointsFrame(uint8_t *data, uint32_t dataBufSize, FrameBufferReclaim customBufferReclaim = nullptr);

    void  setPositionValueScale(float valueScale);
    float getPositionValueScale() const;

private:
    float valueScale_;
};

class AccelFrame : public Frame {
public:
    typedef struct {
        float accelData[3];  // Acceleration values ​​in three directions (xyz), unit: g
        float temp;          // Celsius
    } OBAccelFrameData;

public:
    AccelFrame(uint8_t *data, uint32_t dataBufSize, FrameBufferReclaim customBufferReclaim = nullptr);

    OBAccelValue value();
    float        temperature();
};

class GyroFrame : public Frame {
public:
    typedef struct {
        float gyroData[3];  // Acceleration values ​​in three directions (xyz), unit: dps
        float temp;         // Celsius
    } OBGyroFrameData;

public:
    GyroFrame(uint8_t *data, uint32_t dataBufSize, FrameBufferReclaim customBufferReclaim = nullptr);

    OBGyroValue value();
    float       temperature();
};

class RawPhaseFrame : public VideoFrame {
public:
    RawPhaseFrame(uint8_t *data, uint32_t dataBufSize, FrameBufferReclaim customBufferReclaim = nullptr);
};

class FrameSet : public Frame {
    typedef std::function<bool(void *)> ForeachBack;

public:
    FrameSet(uint8_t *data, uint32_t dataBufSize, FrameBufferReclaim customBufferReclaim = nullptr);
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
    // void pushFrame(OBFrameType type, std::shared_ptr<Frame> frame);

    // Note that when using pushFrame(frame->type(), std::move(frame)), some compilers will first execute std::move(frame) after compilation.
    // Execute frame->type() again, causing the program to crash when frame->type()
    void pushFrame(OBFrameType type, std::shared_ptr<Frame> &&frame);  // todo: delete this function
    void pushFrame(std::shared_ptr<Frame> &&frame);
    void clearAllFrame();

public:
    void foreachFrame(ForeachBack foreachBack);
};

template <typename T> bool Frame::is() {
    switch(type_) {

    case OB_FRAME_VIDEO:
        return (typeid(T) == typeid(IRFrame) || typeid(T) == typeid(DepthFrame) || typeid(T) == typeid(ColorFrame) || typeid(T) == typeid(RawPhaseFrame)
                || typeid(T) == typeid(VideoFrame));
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
    case OB_FRAME_RAW_PHASE:
        return (typeid(T) == typeid(RawPhaseFrame) || typeid(T) == typeid(VideoFrame));
    default:
        break;
    }
    return false;
}

uint32_t calcVideoFrameMaxDataSize(OBFormat format, uint32_t width, uint32_t height);

}  // namespace core
}  // namespace ob
