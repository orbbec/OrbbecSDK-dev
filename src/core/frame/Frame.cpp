#include "Frame.hpp"
#include "logger/Logger.hpp"
#include "utils/StringUtils.hpp"

namespace ob {
namespace core {

Frame::Frame(uint8_t *data, uint32_t dataBufSize, OBFrameType type, FrameBufferReclaim customBufferReclaim)
    : frameData_(data), dataBufSize_(dataBufSize), type_(type), customBufferReclaim_(customBufferReclaim) {}

Frame::~Frame() noexcept {
    if(customBufferReclaim_) {
        customBufferReclaim_();
    }
    else {
        delete[] frameData_;
    }
}

OBFrameType Frame::getType() {
    return type_;
}

OBFormat Frame::getFormat() {
    return format_;
}

void Frame::setFormat(const OBFormat format) {
    format_ = format;
}

uint32_t Frame::getFps() {
    return fps_;
}

void Frame::setFps(const uint32_t fps) {
    fps_ = fps;
}

uint32_t Frame::getNumber() {
    return number_;
}

void Frame::setNumber(const uint32_t number) {
    number_ = number;
}

uint32_t Frame::getDataSize() {
    return dataSize_;
}

void Frame::setDataSize(uint32_t dataSize) {
    dataSize_ = dataSize;
}

uint8_t *Frame::getData() {
    return frameData_;
}

void Frame::updateData(const uint8_t *data, uint32_t dataSize) {
    if(dataSize > dataBufSize_) {
        throw memory_exception(utils::to_string() << "Update data size(" << dataSize << ") > data buffer size! (" << dataBufSize_ << ")");
    }
    dataSize_ = dataSize;
    memcpy(frameData_, data, dataSize);
}

uint64_t Frame::getTimeStamp() {
    return timeStampUsec_ / 1000.0;
}

uint64_t Frame::getTimeStampUs() {
    return timeStampUsec_;
}

void Frame::setTimeStampUs(const uint64_t ts) {
    timeStampUsec_ = ts;
}

uint64_t Frame::getSystemTimeStamp() {
    return systemTimeStampUsec_ / 1000.0;
}

uint64_t Frame::getSystemTimeStampUs() {
    return systemTimeStampUsec_;
}

void Frame::setSystemTimeStampUs(const uint64_t ts) {
    systemTimeStampUsec_ = ts;
}

uint64_t Frame::getGlobalTimeStampUs() {
    return globalTimeStampUsec_;
}

void Frame::setGlobalTimeStampUS(const uint64_t ts) {
    globalTimeStampUsec_ = ts;
}

uint32_t Frame::getWidth() {
    return width_;
}

void Frame::setWidth(uint32_t width) {
    width_ = width;
}

uint32_t Frame::getHeight() {
    return height_;
}

void Frame::setHeight(uint32_t height) {
    height_ = height;
}

uint32_t Frame::getStride() {
    if(stride_ == 0) {
        switch(format_) {
        case OB_FORMAT_Y8:
        case OB_FORMAT_BA81:
            stride_ = width_;
            break;
        case OB_FORMAT_Y10:
            stride_ = width_ * 8 / 10;
            break;
        case OB_FORMAT_Y11:
            stride_ = width_ * 8 / 11;
            break;
        case OB_FORMAT_Y12:
        case OB_FORMAT_NV12:
        case OB_FORMAT_YV12:
            stride_ = width_ * 8 / 12;
            break;
        case OB_FORMAT_Y14:
            stride_ = width_ * 8 / 14;
            break;
        case OB_FORMAT_Y16:
        case OB_FORMAT_Z16:
        case OB_FORMAT_YUYV:
        case OB_FORMAT_UYVY:
        case OB_FORMAT_BYR2:
        case OB_FORMAT_RW16:
        case OB_FORMAT_DISP16:
            stride_ = width_ * 2;
            break;
        case OB_FORMAT_RGB:
        case OB_FORMAT_BGR:
            stride_ = width_ * 3;
            break;
        case OB_FORMAT_POINT:
            stride_ = width_ * 12;
            break;
        case OB_FORMAT_RGB_POINT:
            stride_ = width_ * 24;
            break;
        default:
            throw std::runtime_error("Get stride bytes failed! Unsupported operation for codec format and (semi-)planar packed format object");
            break;
        }
    }
    return stride_;
}

void Frame::setStride(uint32_t stride) {
    stride_ = stride;
}

uint32_t Frame::getMetadataSize() const {
    return metadataSize_;
}

void Frame::updateMetadata(const uint8_t *metadata, uint32_t metadataSize) {

    if(metadataSize > 0 && metadata == nullptr) {
        // In the try_read_metadata() function, metadata may be empty.
        throw memory_exception("Metadata is null!");
    }
    metadataSize_ = metadataSize;
    memcpy(metadata_, metadata, metadataSize);
}

uint8_t *Frame::getMetadata() {
    return metadata_;
}

void Frame::registerMetadataParsers(std::shared_ptr<IFrameMetadataParserContainer> parsers) {
    metadataPhasers_ = parsers;
}

bool Frame::hasMetadata(OBFrameMetadataType type) {
    if(!metadataPhasers_) {
        return false;
    }
    if(!metadataPhasers_->isContained(type)) {
        return false;
    }
    auto parser = metadataPhasers_->get(type);
    return parser->isSupported(metadata_, metadataSize_);
}

int64_t Frame::getMetadataValue(OBFrameMetadataType type) {
    if(!metadataPhasers_) {
        throw unsupported_operation_exception(utils::to_string() << "Unsupported metadata type: " << type);
    }
    auto parser = metadataPhasers_->get(type);
    return parser->getValue(metadata_, metadataSize_);
}

std::shared_ptr<const StreamProfile> Frame::getStreamProfile() const {
    return streamProfile_;
}

void Frame::setStreamProfile(std::shared_ptr<const StreamProfile> streamProfile) {
    streamProfile_ = streamProfile;
}

uint32_t Frame::getBytesPerPixel() {
    // bytes-per-pixel value
    uint32_t nBytesPerPixel = 0;
    switch(format_) {
    case OB_FORMAT_Y8:
    case OB_FORMAT_BA81:
        nBytesPerPixel = sizeof(uint8_t);
        break;
    case OB_FORMAT_Y10:
        nBytesPerPixel = sizeof(uint8_t) * 2;
        break;
    case OB_FORMAT_Y11:
        nBytesPerPixel = sizeof(uint8_t) * 2;
        break;
    case OB_FORMAT_Y12:
    case OB_FORMAT_NV12:
    case OB_FORMAT_YV12:
        nBytesPerPixel = sizeof(uint8_t) * 2;
        break;
    case OB_FORMAT_Y14:
        nBytesPerPixel = sizeof(uint8_t) * 2;
        break;
    case OB_FORMAT_Y16:
    case OB_FORMAT_Z16:
    case OB_FORMAT_YUYV:
    case OB_FORMAT_UYVY:
    case OB_FORMAT_BYR2:
    case OB_FORMAT_RW16:
    case OB_FORMAT_DISP16:
        nBytesPerPixel = sizeof(uint8_t) * 2;
        break;
    case OB_FORMAT_RGB:
    case OB_FORMAT_BGR:
        nBytesPerPixel = sizeof(uint8_t) * 3;
        break;
    case OB_FORMAT_RGBA:
    case OB_FORMAT_BGRA:
        nBytesPerPixel = sizeof(uint8_t) * 4;
        break;
    case OB_FORMAT_POINT:
        nBytesPerPixel = sizeof(uint8_t) * 12;
        break;
    case OB_FORMAT_RGB_POINT:
        nBytesPerPixel = sizeof(uint8_t) * 24;
        break;
    default:
        throw std::runtime_error("Get bytes per pixel value failed!");
        break;
    }

    return nBytesPerPixel;
}

void Frame::copyInfo(const std::shared_ptr<Frame> sourceFrame) {
    // type_ = sourceFrame->type_; //type is determined during construction. It is an inherent property of the object and cannot be changed.

    fps_                 = sourceFrame->fps_;
    number_              = sourceFrame->number_;
    timeStampUsec_       = sourceFrame->timeStampUsec_;
    systemTimeStampUsec_ = sourceFrame->systemTimeStampUsec_;
    globalTimeStampUsec_ = sourceFrame->globalTimeStampUsec_;
    format_              = sourceFrame->format_;
    width_               = sourceFrame->width_;
    height_              = sourceFrame->height_;
    stride_              = sourceFrame->stride_;
    dataSize_            = sourceFrame->dataSize_;
    streamProfile_       = sourceFrame->streamProfile_;

    metadataSize_ = sourceFrame->metadataSize_;
    memcpy(metadata_, sourceFrame->metadata_, metadataSize_);
    metadataPhasers_ = sourceFrame->metadataPhasers_;
}

std::shared_ptr<Frame> Frame::convertTo(OBFrameType tarFrameType) {
    // Convert the object type, but the buffer and buffer recycling function need to be transferred
    std::shared_ptr<Frame> targetFrame;
    if(!is<VideoFrame>()) {
        throw unsupported_operation_exception("Unsupported source Type yet!");
    }
    switch(tarFrameType) {
    case OB_FRAME_DEPTH:
        targetFrame = std::make_shared<DepthFrame>(frameData_, dataBufSize_, customBufferReclaim_);
        break;
    case OB_FRAME_IR:
        targetFrame = std::make_shared<IRFrame>(frameData_, dataBufSize_, customBufferReclaim_);
        break;
    case OB_FRAME_COLOR:
        targetFrame = std::make_shared<ColorFrame>(frameData_, dataBufSize_, customBufferReclaim_);
        break;
    default:
        throw unsupported_operation_exception("Unsupported target Type yet!");
        break;
    }

    targetFrame->copyInfo(shared_from_this());

    customBufferReclaim_ = []() {
        // do nothing;
    };
    return targetFrame;
}

uint32_t Frame::getDataBufSize() const {
    return dataBufSize_;
}

VideoFrame::VideoFrame(uint8_t *data, uint32_t dataBufSize, OBFrameType type, FrameBufferReclaim customBufferReclaim)
    : Frame(data, dataBufSize, type, customBufferReclaim), availableBitSize_(0) {}

void VideoFrame::setFormat(const OBFormat format) {
    format_ = format;
    if(availableBitSize_ == 0) {
        switch(format_) {
        case OB_FORMAT_Y8:
        case OB_FORMAT_BA81:
            availableBitSize_ = 8;
            break;
        case OB_FORMAT_Y10:
            availableBitSize_ = 10;
            break;
        case OB_FORMAT_Y11:
            availableBitSize_ = 11;
            break;
        case OB_FORMAT_Y12:
        case OB_FORMAT_YV12:
        case OB_FORMAT_NV12:
            availableBitSize_ = 12;
            break;
        case OB_FORMAT_Y14:
            availableBitSize_ = 14;
            break;
        case OB_FORMAT_RLE:
            availableBitSize_ = 14;  // todo: RLE may not be packaged in 14bit
            break;
        case OB_FORMAT_RVL:
            availableBitSize_ = 16;
            break;
        case OB_FORMAT_Y16:
        case OB_FORMAT_Z16:
        case OB_FORMAT_YUYV:
        case OB_FORMAT_UYVY:
        case OB_FORMAT_BYR2:
        case OB_FORMAT_RW16:
        case OB_FORMAT_DISP16:
            availableBitSize_ = 16;  // The actual number of effective digits varies from device to device and requires Frame external settings.
            break;
        default:
            break;
        }
    }
}

uint8_t VideoFrame::getScrDataSize() const {
    return scrDataSize_;
}

void VideoFrame::updateScrData(const uint8_t *scrData, uint8_t scrDataSize) {
    if(scrDataSize > 0 && scrData == nullptr) {
        LOG_WARN("scrData is null!");
    }
    scrDataSize_ = scrDataSize;
    memcpy(scrData_, scrData, scrDataSize);
}

uint8_t *VideoFrame::getScrData() {
    return scrData_;
}

uint8_t VideoFrame::getPixelAvailableBitSize() const {
    if(availableBitSize_ == 0) {
        // However, for Depth/Ir non-encoded format, it must be able to return
        LOG_WARN("Unknown pixel available bit size!");
    }
    return availableBitSize_;
}

void VideoFrame::setPixelAvailableBitSize(uint8_t bitSize) {
    availableBitSize_ = bitSize;
}

void VideoFrame::copyInfo(std::shared_ptr<Frame> sourceFrame) {
    Frame::copyInfo(sourceFrame);
    if(sourceFrame->is<VideoFrame>()) {
        availableBitSize_ = sourceFrame->as<VideoFrame>()->availableBitSize_;
    }
}

ColorFrame::ColorFrame(uint8_t *data, uint32_t dataBufSize, FrameBufferReclaim customBufferReclaim)
    : VideoFrame(data, dataBufSize, OB_FRAME_COLOR, customBufferReclaim) {}

DepthFrame::DepthFrame(uint8_t *data, uint32_t dataBufSize, FrameBufferReclaim customBufferReclaim)
    : VideoFrame(data, dataBufSize, OB_FRAME_DEPTH, customBufferReclaim), valueScale_(1.0f) {}

void DepthFrame::setValueScale(float valueScale) {
    valueScale_ = valueScale;
}

float DepthFrame::getValueScale() const {
    return valueScale_;
}

void DepthFrame::copyInfo(std::shared_ptr<Frame> sourceFrame) {
    VideoFrame::copyInfo(sourceFrame);
    if(sourceFrame->is<DepthFrame>()) {
        valueScale_ = sourceFrame->as<DepthFrame>()->valueScale_;
    }
}

IRFrame::IRFrame(uint8_t *data, uint32_t dataBufSize, FrameBufferReclaim customBufferReclaim, OBFrameType frameType)
    : VideoFrame(data, dataBufSize, frameType, customBufferReclaim) {}
IRLeftFrame::IRLeftFrame(uint8_t *data, uint32_t dataBufSize, FrameBufferReclaim customBufferReclaim)
    : IRFrame(data, dataBufSize, customBufferReclaim, OB_FRAME_IR_LEFT) {}
IRRightFrame::IRRightFrame(uint8_t *data, uint32_t dataBufSize, FrameBufferReclaim customBufferReclaim)
    : IRFrame(data, dataBufSize, customBufferReclaim, OB_FRAME_IR_RIGHT) {}

PointsFrame::PointsFrame(uint8_t *data, uint32_t dataBufSize, FrameBufferReclaim customBufferReclaim)
    : Frame(data, dataBufSize, OB_FRAME_POINTS, customBufferReclaim) {}

void PointsFrame::setPositionValueScale(float valueScale) {
    valueScale_ = valueScale;
}

float PointsFrame::getPositionValueScale() const {
    return valueScale_;
}

AccelFrame::AccelFrame(uint8_t *data, uint32_t dataBufSize, FrameBufferReclaim customBufferReclaim)
    : Frame(data, dataBufSize, OB_FRAME_ACCEL, customBufferReclaim) {}

OBAccelValue AccelFrame::value() {
    return *(OBAccelValue *)getData();
}

float AccelFrame::temperature() {
    return ((OBAccelFrameData *)getData())->temp;
}

GyroFrame::GyroFrame(uint8_t *data, uint32_t dataBufSize, FrameBufferReclaim customBufferReclaim)
    : Frame(data, dataBufSize, OB_FRAME_GYRO, customBufferReclaim) {}

OBGyroValue GyroFrame ::value() {
    return *(OBGyroValue *)getData();
}

float GyroFrame ::temperature() {
    return ((OBGyroFrameData *)getData())->temp;
}

RawPhaseFrame::RawPhaseFrame(uint8_t *data, uint32_t dataBufSize, FrameBufferReclaim customBufferReclaim)
    : VideoFrame(data, dataBufSize, OB_FRAME_RAW_PHASE, customBufferReclaim) {}

FrameSet::FrameSet(uint8_t *data, uint32_t dataBufSize, FrameBufferReclaim customBufferReclaim) : Frame(data, dataBufSize, OB_FRAME_SET, customBufferReclaim) {}

FrameSet::~FrameSet() {
    clearAllFrame();
}

uint32_t FrameSet::getFrameCount() {
    uint32_t frameCnt = 0;
    foreachFrame([&](void *item) {
        std::shared_ptr<Frame> *pFrame = (std::shared_ptr<Frame> *)item;
        if(*pFrame) {
            frameCnt++;
        }
        return false;
    });
    return frameCnt;
}

std::shared_ptr<Frame> FrameSet::getDepthFrame() {
    std::shared_ptr<Frame> frame;
    foreachFrame([&](void *item) {
        std::shared_ptr<Frame> *pFrame = (std::shared_ptr<Frame> *)item;
        if(*pFrame && (*pFrame)->getType() == OB_FRAME_DEPTH) {
            frame = *pFrame;
            return true;
        }
        return false;
    });
    return frame;
}

std::shared_ptr<Frame> FrameSet::getIRFrame() {
    std::shared_ptr<Frame> frame;
    foreachFrame([&](void *item) {
        std::shared_ptr<Frame> *pFrame = (std::shared_ptr<Frame> *)item;
        if(*pFrame && (*pFrame)->getType() == OB_FRAME_IR) {
            frame = *pFrame;
            return true;
        }
        return false;
    });
    return frame;
}

std::shared_ptr<Frame> FrameSet::getColorFrame() {
    std::shared_ptr<Frame> frame;
    foreachFrame([&](void *item) {
        std::shared_ptr<Frame> *pFrame = (std::shared_ptr<Frame> *)item;
        if(*pFrame && (*pFrame)->getType() == OB_FRAME_COLOR) {
            frame = *pFrame;
            return true;
        }
        return false;
    });
    return frame;
}

std::shared_ptr<Frame> FrameSet::getAccelFrame() {
    std::shared_ptr<Frame> frame;
    foreachFrame([&](void *item) {
        std::shared_ptr<Frame> *pFrame = (std::shared_ptr<Frame> *)item;
        if(*pFrame && (*pFrame)->getType() == OB_FRAME_ACCEL) {
            frame = *pFrame;
            return true;
        }
        return false;
    });
    return frame;
}

std::shared_ptr<Frame> FrameSet::getGyroFrame() {
    std::shared_ptr<Frame> frame;
    foreachFrame([&](void *item) {
        std::shared_ptr<Frame> *pFrame = (std::shared_ptr<Frame> *)item;
        if(*pFrame && (*pFrame)->getType() == OB_FRAME_GYRO) {
            frame = *pFrame;
            return true;
        }
        return false;
    });
    return frame;
}

std::shared_ptr<Frame> FrameSet::getPointsFrame() {
    std::shared_ptr<Frame> frame;
    foreachFrame([&](void *item) {
        std::shared_ptr<Frame> *pFrame = (std::shared_ptr<Frame> *)item;
        if(*pFrame && (*pFrame)->getType() == OB_FRAME_POINTS) {
            frame = *pFrame;
            return true;
        }
        return false;
    });
    return frame;
}

std::shared_ptr<Frame> FrameSet::getFrame(OBFrameType frameType) {
    std::shared_ptr<Frame> frame;
    foreachFrame([&](void *item) {
        std::shared_ptr<Frame> *pFrame = (std::shared_ptr<Frame> *)item;
        if(*pFrame && (*pFrame)->getType() == frameType) {
            frame = *pFrame;
            return true;
        }
        return false;
    });
    return frame;
}

std::shared_ptr<Frame> FrameSet::getFrame(int index) {
    std::shared_ptr<Frame> frame;
    uint32_t               itemSize = sizeof(std::shared_ptr<Frame>);
    auto                   itemCnt  = getDataBufSize() / itemSize;
    auto                   pItem    = getData();
    pItem += itemSize * index;
    std::shared_ptr<Frame> *pFrame = (std::shared_ptr<Frame> *)pItem;
    frame                          = *pFrame;
    return frame;
}

// It is recommended to use the rvalue reference interface. If you have a need, you can uncomment the following
// void FrameSet::pushFrame(ob_frame_type type, std::shared_ptr<Frame> frame) {
// pushFrame(std::move(frame));
// }

// Note that when using pushFrame(frame->type(), std::move(frame)), some compilers will first execute std::move(frame) after compilation.
// Executing frame->type() again will cause the program to crash when frame->type() is executed.
void FrameSet::pushFrame(OBFrameType type, std::shared_ptr<Frame> &&frame) {
    foreachFrame([&](void *item) {
        std::shared_ptr<Frame> *pFrame = (std::shared_ptr<Frame> *)item;
        if(*pFrame && (*pFrame)->getType() == type) {
            (*pFrame).reset();
            *pFrame = nullptr;
            // LOG( ERROR ) << "The reason for dropping frames is to wait for another frame. Drop Frame type=" << type;
        }
        if(!(*pFrame)) {
            *pFrame = frame;
            return true;
        }
        return false;
    });
}

void FrameSet::pushFrame(std::shared_ptr<Frame> &&frame) {
    OBFrameType type = frame->getType();
    foreachFrame([&](void *item) {
        std::shared_ptr<Frame> *pFrame = (std::shared_ptr<Frame> *)item;
        if(*pFrame && (*pFrame)->getType() == type) {
            (*pFrame).reset();
            *pFrame = nullptr;
            // LOG( ERROR ) << "The reason for dropping frames is to wait for another frame. Drop Frame type=" << type;
        }
        if(!(*pFrame)) {
            *pFrame = frame;
            return true;
        }
        return false;
    });
}

void FrameSet::clearAllFrame() {
    foreachFrame([](void *item) {
        std::shared_ptr<Frame> *pFrame = (std::shared_ptr<Frame> *)item;
        if(*pFrame) {
            (*pFrame).reset();
        }
        *pFrame = nullptr;
        return false;
    });
}

void FrameSet::foreachFrame(ForeachBack foreachBack) {
    uint32_t itemSize = sizeof(std::shared_ptr<Frame>);
    auto     itemCnt  = getDataBufSize() / itemSize;
    auto     pItem    = getData();
    for(int i = 0; i < itemCnt; i++) {
        if(foreachBack(pItem)) {
            break;
        }
        pItem += itemSize;
    }
}

uint32_t calcVideoFrameMaxDataSize(OBFormat format, uint32_t width, uint32_t height) {
    uint32_t maxFrameDataSize = height * width * 3;
    switch(format) {
    case OB_FORMAT_NV21:
    case OB_FORMAT_I420:
        maxFrameDataSize = height * width * 3 / 2;
        break;
    case OB_FORMAT_MJPG:
    case OB_FORMAT_H264:
    case OB_FORMAT_H265:
    case OB_FORMAT_HEVC:
    case OB_FORMAT_Y8:
    case OB_FORMAT_BA81:
        maxFrameDataSize = height * width;
        break;
    case OB_FORMAT_YUYV:
    case OB_FORMAT_UYVY:
    case OB_FORMAT_YUY2:
    case OB_FORMAT_Y16:
    case OB_FORMAT_BYR2:
    case OB_FORMAT_RW16:
    case OB_FORMAT_Z16:
    case OB_FORMAT_RLE:
    case OB_FORMAT_RVL:
    case OB_FORMAT_DISP16:
        maxFrameDataSize = height * width * 2;
        break;
    case OB_FORMAT_Y10:
        maxFrameDataSize = height * width * 10 / 8 + 0.5;
        break;
    case OB_FORMAT_Y11:
        maxFrameDataSize = height * width * 11 / 8 + 0.5;
        break;
    case OB_FORMAT_Y12:
    case OB_FORMAT_NV12:
    case OB_FORMAT_YV12:
        maxFrameDataSize = height * width * 12 / 8 + 0.5;
        break;
    case OB_FORMAT_Y14:
        maxFrameDataSize = height * width * 14 / 8 + 0.5;
        break;
    case OB_FORMAT_RGBA:
    case OB_FORMAT_BGRA:
        maxFrameDataSize = width * height * 4;
        break;
    case OB_FORMAT_RGB:
    case OB_FORMAT_BGR:
        maxFrameDataSize = width * height * 3;
        break;
    default:
        LOG_WARN("Unknown video frame format!");
        maxFrameDataSize = width * height * 3;
        break;
    }

    return maxFrameDataSize;
}

}  // namespace core
}  // namespace ob
