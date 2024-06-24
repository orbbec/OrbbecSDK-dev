#include "DU08mmTo1mmConverter.hpp"
#include "exception/ObException.hpp"
#include "logger/LoggerInterval.hpp"
#include "frame/FrameFactory.hpp"
#include "libobsensor/h/ObTypes.h"

namespace libobsensor {

DU08mmTo1mmConverter::DU08mmTo1mmConverter(const std::string &name) : FilterBase(name) {}
DU08mmTo1mmConverter::~DU08mmTo1mmConverter() noexcept {}

void DU08mmTo1mmConverter::updateConfig(std::vector<std::string> &params) {
    if(params.size() != 0) {
        throw unsupported_operation_exception("DU08mmTo1mmConverter update config error: unsupported operation.");
    }
}

const std::string &DU08mmTo1mmConverter::getConfigSchema() const {
    throw unsupported_operation_exception("DU08mmTo1mmConverter get config schema error: unsupported operation.");
}

std::shared_ptr<Frame> DU08mmTo1mmConverter::processFunc(std::shared_ptr<const Frame> frame) {
    if(!frame) {
        return nullptr;
    }

    auto newFrame = FrameFactory::cloneFrame(frame);
    if(frame->is<FrameSet>()) {
        LOG_WARN_INTVL("The Frame processed by DU08mmTo1mmConverter cannot be FrameSet!");
        return newFrame;
    }

    auto outFrame = newFrame->as<VideoFrame>();
    if(outFrame->getWidth() * outFrame->getHeight() == outFrame->getDataSize()) {
        for(size_t i = 0; i < outFrame->getDataSize(); i++) {
            uint8_t *value = (uint8_t *)outFrame->getData() + i;
            if(*value > 63) {
                *value = *value / 5 * 4;
            }
            else {
                *value = *value * 4 / 5;
            }
        }
    }
    else if(outFrame->getWidth() * outFrame->getHeight() * 2 == outFrame->getDataSize()) {
        for(size_t i = 0, N = outFrame->getDataSize() / 2; i < N; i++) {
            uint16_t *value = (uint16_t *)outFrame->getData() + i;
            if(*value > 16383) {
                *value = *value / 5 * 4;
            }
            else {
                *value = *value * 4 / 5;
            }
        }
    }

    return outFrame;
}

}  // namespace libobsensor
