#include "frame/FrameMemoryPool.hpp"
#include "logger/Logger.hpp"

using namespace ob::core;
int main(int argc, char **argv) {
    auto logger                  = ob::Logger();
    logger.setLogSeverity(OB_LOG_SEVERITY_DEBUG);
    auto frameMemoryPool         = FrameMemoryPool::getInstance();
    auto depthFrameBufferManager = frameMemoryPool->createFrameBufferManager(OB_FRAME_DEPTH, 640 * 480 * 2);
    auto depthFrame              = depthFrameBufferManager->acquireFrame();

    auto data     = depthFrame->getData();
    auto dataSize = depthFrame->getDataSize();
    LOG_INFO("data ptr: {}, size: {}", (uint64_t)data, dataSize);

    depthFrame.reset();
    depthFrameBufferManager.reset();
    frameMemoryPool.reset();
    FrameMemoryPool::releaseInstance();
    return 0;
}