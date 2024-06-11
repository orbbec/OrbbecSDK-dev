#include "FramePixelValueProcess.hpp"
#include "exception/ObException.hpp"
#include "logger/LoggerInterval.hpp"
#include "frame/FrameFactory.hpp"

namespace libobsensor {

template <typename T> void imagePixelValueScale(const T *src, T *dst, uint32_t width, uint32_t height, float scale_) {
    const T *srcPixel = src;
    T       *dstPixel = dst;
    for(uint32_t h = 0; h < height; h++) {
        for(uint32_t w = 0; w < width; w++) {
            *dstPixel = (T)(*srcPixel * scale_);
            srcPixel++;
            dstPixel++;
        }
    }
}

template <typename T> void imagePixelValueOffset(T *src, T *dst, uint32_t width, uint32_t height, int8_t offset) {
    T *dstPixel = dst;
    T *srcPixel = src;
    if(offset > 0) {
        uint8_t shiftCount = offset;
        for(uint32_t h = 0; h < height; h++) {
            for(uint32_t w = 0; w < width; w++) {
                *dstPixel = *srcPixel >> shiftCount;
                srcPixel++;
                dstPixel++;
            }
        }
    }
    else if(offset < 0) {
        uint8_t shiftCount = -offset;
        for(uint32_t h = 0; h < height; h++) {
            for(uint32_t w = 0; w < width; w++) {
                *dstPixel = *srcPixel << shiftCount;
                srcPixel++;
                dstPixel++;
            }
        }
    }
}

template <typename T> void imagePixelValueCutOff(T *src, T *dst, uint32_t width, uint32_t height, uint32_t min, uint32_t max) {
    T *dstPixel = dst;
    T *srcPixel = src;
    if(min >= max) {
        for(uint32_t h = 0; h < height; h++) {
            for(uint32_t w = 0; w < width; w++) {
                *dstPixel = 0;
                srcPixel++;
                dstPixel++;
            }
        }
    }
    else {
        for(uint32_t h = 0; h < height; h++) {
            for(uint32_t w = 0; w < width; w++) {
                *dstPixel = (*srcPixel < min) ? 0 : ((*srcPixel > max) ? 0 : *srcPixel);
                srcPixel++;
                dstPixel++;
            }
        }
    }
}

PixelValueScaler::PixelValueScaler(const std::string &name) : FilterBase(name) {}
PixelValueScaler::~PixelValueScaler() noexcept {}

void PixelValueScaler::updateConfig(std::vector<std::string> &params) {
    if(params.size() != 1) {
        throw invalid_value_exception("PixelValueScaler config error: params size not match");
    }
    try {
        scale_ = std::stof(params[0]);
    }
    catch(const std::exception &e) {
        throw invalid_value_exception("PixelValueScaler config error: " + std::string(e.what()));
    }
}

const std::string &PixelValueScaler::getConfigSchema() const {
    // csv format: name，type， min，max，step，default，description
    static const std::string schema = "scale, float, 0.01, 100.0, 0.01, 1.0, value scale factor";
    return schema;
}

std::shared_ptr<Frame> PixelValueScaler::processFunc(std::shared_ptr<const Frame> frame) {
    if(frame->getType() != OB_FRAME_DEPTH) {
        LOG_WARN_INTVL("PixelValueScaler unsupported to process this frame type: {}", frame->getType());
        return FrameFactory::cloneFrame(frame);
    }

    auto depthFrame = frame->as<DepthFrame>();

    auto outFrame = FrameFactory::cloneFrame(frame);

    switch(frame->getFormat()) {
    case OB_FORMAT_Y16:
        imagePixelValueScale<uint16_t>((uint16_t *)frame->getData(), (uint16_t *)outFrame->getData(), depthFrame->getWidth(), depthFrame->getHeight(), scale_);
        break;
    default:
        break;
    }

    auto outDepthFrame = outFrame->as<DepthFrame>();
    auto valueScale    = outDepthFrame->getValueScale();
    outDepthFrame->setValueScale(valueScale * scale_);

    return outFrame;
}
}  // namespace libobsensor
