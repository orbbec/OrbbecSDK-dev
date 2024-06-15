#include "Frame.hpp"
#include "logger/Logger.hpp"
#include "utils/Utils.hpp"
#include "stream/StreamProfile.hpp"

namespace libobsensor {

Frame::Frame(uint8_t *data, size_t dataBufSize, OBFrameType type, FrameBufferReclaimFunc bufferReclaimFunc)
    : dataSize_(dataBufSize),
      number_(0),
      timeStampUsec_(0),
      systemTimeStampUsec_(0),
      globalTimeStampUsec_(0),
      metadataSize_(0),
      metadataPhasers_(nullptr),
      streamProfile_(nullptr),
      type_(type),
      frameData_(data),
      dataBufSize_(dataBufSize),
      bufferReclaimFunc_(bufferReclaimFunc) {}


Frame::~Frame() noexcept {
    if(bufferReclaimFunc_) {
        bufferReclaimFunc_();
    }
    else {
        delete[] frameData_;
    }
}

OBFrameType Frame::getType() const {
    return type_;
}

OBFormat Frame::getFormat() const {
    if(!streamProfile_) {
        return OB_FORMAT_UNKNOWN;
    }
    return streamProfile_->getFormat();
}

uint64_t Frame::getNumber() const {
    return number_;
}

void Frame::setNumber(const uint64_t number) {
    number_ = number;
}

size_t Frame::getDataSize() const {
    return dataSize_;
}

const uint8_t *Frame::getData() const {
    return frameData_;
}

uint8_t *Frame::getDataUnsafe() const {
    return const_cast<uint8_t *>(frameData_);
}

void Frame::updateData(const uint8_t *data, size_t dataSize) {
    if(dataSize > dataBufSize_) {
        throw memory_exception(utils::to_string() << "Update data size(" << dataSize << ") > data buffer size! (" << dataBufSize_ << ")");
    }
    dataSize_ = dataSize;
    memcpy(const_cast<uint8_t *>(frameData_), data, dataSize);
}

uint64_t Frame::getTimeStampUsec() const {
    return timeStampUsec_;
}

void Frame::setTimeStampUsec(uint64_t ts) {
    timeStampUsec_ = ts;
}

uint64_t Frame::getSystemTimeStampUsec() const {
    return systemTimeStampUsec_;
}

void Frame::setSystemTimeStampUsec(uint64_t ts) {
    systemTimeStampUsec_ = ts;
}

uint64_t Frame::getGlobalTimeStampUsec() const {
    return globalTimeStampUsec_;
}

void Frame::setGlobalTimeStampUsec(uint64_t ts) {
    globalTimeStampUsec_ = ts;
}

uint32_t VideoFrame::getFps() const {
    if(!streamProfile_) {
        throw invalid_value_exception("Error: This frame dose not have a stream profile!");
    }
    if(!streamProfile_->is<VideoStreamProfile>()) {
        throw invalid_value_exception("Error! A VideoFrame contain a non-video stream profile!");
    }
    return streamProfile_->as<VideoStreamProfile>()->getFps();
}

uint32_t VideoFrame::getWidth() const {
    if(!streamProfile_) {
        throw invalid_value_exception("Error: This frame dose not have a stream profile!");
    }
    if(!streamProfile_->is<const VideoStreamProfile>()) {
        throw invalid_value_exception("Error! A VideoFrame contain a non-video stream profile!");
    }
    return streamProfile_->as<VideoStreamProfile>()->getWidth();
}

uint32_t VideoFrame::getHeight() const {
    if(!streamProfile_) {
        throw invalid_value_exception("Error: This frame dose not have a stream profile!");
    }
    if(!streamProfile_->is<VideoStreamProfile>()) {
        throw invalid_value_exception("Error! A VideoFrame contain a non-video stream profile!");
    }
    return streamProfile_->as<VideoStreamProfile>()->getHeight();
}

uint32_t VideoFrame::getStride() const {
    if(stride_ > 0) {
        return stride_;
    }
    auto format = getFormat();
    auto width  = getWidth();
    return utils::calcDefaultStrideBytes(format, width);
}

void VideoFrame::setStride(uint32_t stride) {
    stride_ = stride;
}

size_t Frame::getMetadataSize() const {
    return metadataSize_;
}

void Frame::updateMetadata(const uint8_t *metadata, size_t metadataSize) {
    if(metadataSize > 0 && metadata == nullptr) {
        // In the try_read_metadata() function, metadata may be empty.
        throw memory_exception("Metadata is null!");
    }
    if(metadataSize > sizeof(metadata_)) {
        throw memory_exception("Metadata size is too large!");
    }
    metadataSize_ = metadataSize;
    memcpy(metadata_, metadata, metadataSize);
}

const uint8_t *Frame::getMetadata() const {
    return metadata_;
}

void Frame::registerMetadataParsers(std::shared_ptr<IFrameMetadataParserContainer> parsers) {
    metadataPhasers_ = parsers;
}

bool Frame::hasMetadata(OBFrameMetadataType type) const {
    if(!metadataPhasers_) {
        return false;
    }
    if(!metadataPhasers_->isContained(type)) {
        return false;
    }
    auto parser = metadataPhasers_->get(type);
    return parser->isSupported(metadata_, metadataSize_);
}

int64_t Frame::getMetadataValue(OBFrameMetadataType type) const {
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

void Frame::copyInfo(const std::shared_ptr<const Frame> sourceFrame) {
    // type is determined during construction. It is an inherent property of the object and cannot be changed.
    // type_ = sourceFrame->type_;

    // todo: check if it is necessary to copy those properties.

    number_              = sourceFrame->number_;
    timeStampUsec_       = sourceFrame->timeStampUsec_;
    systemTimeStampUsec_ = sourceFrame->systemTimeStampUsec_;
    globalTimeStampUsec_ = sourceFrame->globalTimeStampUsec_;
    dataSize_            = sourceFrame->dataSize_;
    streamProfile_       = sourceFrame->streamProfile_;

    metadataSize_ = sourceFrame->metadataSize_;
    memcpy(metadata_, sourceFrame->metadata_, metadataSize_);
    metadataPhasers_ = sourceFrame->metadataPhasers_;
}

size_t Frame::getDataBufSize() const {
    return dataBufSize_;
}

VideoFrame::VideoFrame(uint8_t *data, size_t dataBufSize, OBFrameType type, FrameBufferReclaimFunc bufferReclaimFunc)
    : Frame(data, dataBufSize, type, bufferReclaimFunc), availableBitSize_(0) {}

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

void VideoFrame::copyInfo(std::shared_ptr<const Frame> sourceFrame) {
    Frame::copyInfo(sourceFrame);
    if(sourceFrame->is<VideoFrame>()) {
        auto vf           = sourceFrame->as<VideoFrame>();
        availableBitSize_ = vf->availableBitSize_;
        stride_           = vf->stride_;
    }
}

ColorFrame::ColorFrame(uint8_t *data, size_t dataBufSize, FrameBufferReclaimFunc bufferReclaimFunc)
    : VideoFrame(data, dataBufSize, OB_FRAME_COLOR, bufferReclaimFunc) {}

DepthFrame::DepthFrame(uint8_t *data, size_t dataBufSize, FrameBufferReclaimFunc bufferReclaimFunc)
    : VideoFrame(data, dataBufSize, OB_FRAME_DEPTH, bufferReclaimFunc), valueScale_(1.0f) {}

void DepthFrame::setValueScale(float valueScale) {
    valueScale_ = valueScale;
}

float DepthFrame::getValueScale() const {
    return valueScale_;
}

void DepthFrame::copyInfo(std::shared_ptr<const Frame> sourceFrame) {
    VideoFrame::copyInfo(sourceFrame);
    if(sourceFrame->is<DepthFrame>()) {
        auto df     = sourceFrame->as<DepthFrame>();
        valueScale_ = df->valueScale_;
    }
}

DisparityFrame::DisparityFrame(uint8_t *data, size_t dataBufSize, FrameBufferReclaimFunc bufferReclaimFunc)
    : VideoFrame(data, dataBufSize, OB_FRAME_DISPARITY, bufferReclaimFunc) {}

IRFrame::IRFrame(uint8_t *data, size_t dataBufSize, FrameBufferReclaimFunc bufferReclaimFunc, OBFrameType frameType)
    : VideoFrame(data, dataBufSize, frameType, bufferReclaimFunc) {}

IRLeftFrame::IRLeftFrame(uint8_t *data, size_t dataBufSize, FrameBufferReclaimFunc bufferReclaimFunc)
    : IRFrame(data, dataBufSize, bufferReclaimFunc, OB_FRAME_IR_LEFT) {}

IRRightFrame::IRRightFrame(uint8_t *data, size_t dataBufSize, FrameBufferReclaimFunc bufferReclaimFunc)
    : IRFrame(data, dataBufSize, bufferReclaimFunc, OB_FRAME_IR_RIGHT) {}

PointsFrame::PointsFrame(uint8_t *data, size_t dataBufSize, FrameBufferReclaimFunc bufferReclaimFunc)
    : Frame(data, dataBufSize, OB_FRAME_POINTS, bufferReclaimFunc) {}

void PointsFrame::setCoordinateValueScale(float valueScale) {
    coordValueScale_ = valueScale;
}

float PointsFrame::getCoordinateValueScale() const {
    return coordValueScale_;
}

AccelFrame::AccelFrame(uint8_t *data, size_t dataBufSize, FrameBufferReclaimFunc bufferReclaimFunc)
    : Frame(data, dataBufSize, OB_FRAME_ACCEL, bufferReclaimFunc) {}

OBAccelValue AccelFrame::value() {
    return *(OBAccelValue *)getData();
}

float AccelFrame::temperature() {
    return ((OBAccelFrameData *)getData())->temp;
}

GyroFrame::GyroFrame(uint8_t *data, size_t dataBufSize, FrameBufferReclaimFunc bufferReclaimFunc)
    : Frame(data, dataBufSize, OB_FRAME_GYRO, bufferReclaimFunc) {}

OBGyroValue GyroFrame ::value() {
    return *(OBGyroValue *)getData();
}

float GyroFrame ::temperature() {
    return ((OBGyroFrameData *)getData())->temp;
}

FrameSet::FrameSet(uint8_t *data, size_t dataBufSize, FrameBufferReclaimFunc bufferReclaimFunc) : Frame(data, dataBufSize, OB_FRAME_SET, bufferReclaimFunc) {}

FrameSet::~FrameSet() noexcept {
    clearAllFrame();
}

uint32_t FrameSet::getFrameCount() {
    uint32_t frameCnt = 0;
    foreachFrame([&](void *item) {
        auto pFrame = (std::shared_ptr<Frame> *)item;
        if(*pFrame) {
            frameCnt++;
        }
        return false;
    });
    return frameCnt;
}

std::shared_ptr<Frame> FrameSet::getDisparityFrame(){
    std::shared_ptr<Frame> frame;
    foreachFrame([&](void *item){
        auto pFrame=(std::shared_ptr<Frame> *)item;
        if(*pFrame && (*pFrame)->getType()==OB_FRAME_DISPARITY){
            frame=*pFrame;
            return true;
        }
        return false;
    });
    return frame;
}

std::shared_ptr<Frame> FrameSet::getDepthFrame() {
    std::shared_ptr<Frame> frame;
    foreachFrame([&](void *item) {
        auto pFrame = (std::shared_ptr<Frame> *)item;
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
        auto pFrame = (std::shared_ptr<Frame> *)item;
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
        auto pFrame = (std::shared_ptr<Frame> *)item;
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
        auto pFrame = (std::shared_ptr<Frame> *)item;
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
        auto *pFrame = (std::shared_ptr<Frame> *)item;
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
        auto *pFrame = (std::shared_ptr<Frame> *)item;
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
        auto pFrame = (std::shared_ptr<Frame> *)item;
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
    if(index >= (int)itemCnt) {
        throw invalid_value_exception("FrameSet::getFrame() index out of range");
    }
    auto pItem = getData();
    pItem += itemSize * index;
    auto pFrame = (std::shared_ptr<Frame> *)pItem;
    frame       = *pFrame;
    return frame;
}

// It is recommended to use the rvalue reference interface. If you have a need, you can uncomment the following
// void FrameSet::pushFrame(ob_frame_type type, std::shared_ptr<Frame> frame) {
// pushFrame(std::move(frame));
// }

void FrameSet::pushFrame(std::shared_ptr<const Frame> &&frame) {
    OBFrameType type = frame->getType();
    foreachFrame([&](void *item) {
        auto pFrame = (std::shared_ptr<const Frame> *)item;
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
        auto pFrame = (std::shared_ptr<Frame> *)item;
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
    auto     pItem    = const_cast<uint8_t *>(getData());
    for(uint32_t i = 0; i < itemCnt; i++) {
        if(foreachBack(pItem)) {
            break;
        }
        pItem += itemSize;
    }
}

}  // namespace libobsensor
