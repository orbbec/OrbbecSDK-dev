#pragma once

#include "frame/Frame.hpp"

namespace libobsensor {
namespace FrameFactory {

std::shared_ptr<Frame> createFrame(OBFrameType frameType, OBFormat frameFormat, size_t datasize);
std::shared_ptr<Frame> createVideoFrame(OBFrameType frameType, OBFormat frameFormat, int width, int height, int strideBytes);

std::shared_ptr<Frame> createFrameFromUserBuffer(OBFrameType frameType, OBFormat format, uint8_t *buffer, uint32_t bufferSize,
                                                 FrameBufferReclaimFunc bufferReclaimFunc);
std::shared_ptr<Frame> createVideoFrameFromUserBuffer(OBFrameType frameType, OBFormat format, uint32_t frameWidth, uint32_t frameHeight, uint8_t *buffer,
                                                      uint32_t bufferSize, FrameBufferReclaimFunc bufferReclaimFunc);

std::shared_ptr<Frame> createFrameSet();

}  // namespace FrameFactory
}  // namespace libobsensor
