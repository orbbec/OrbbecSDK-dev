#include "frame/FrameMemoryPool.hpp"
#include "frame/FrameQueue.hpp"
#include "CoreTypeHelper.hpp"
#include "logger/Logger.hpp"

using namespace ob::core;
int main(int argc, char **argv) {
    ob::type_helper::unusedVar(argc);
    ob::type_helper::unusedVar(argv);

    auto logger                  = ob::Logger();
    logger.setLogSeverity(OB_LOG_SEVERITY_DEBUG);
    auto frameMemoryPool         = FrameMemoryPool::getInstance();
    auto depthFrameBufferManager = frameMemoryPool->createFrameBufferManager(OB_FRAME_DEPTH, 640 * 480 * 2);
    auto depthFrame              = depthFrameBufferManager->acquireFrame();

    depthFrame->updateData((uint8_t*)"test", 4);  // update data, data size must be less than buffer size ( 640 * 480 * 2)

    auto data     = depthFrame->getData();
    auto dataSize = depthFrame->getDataSize();
    LOG_INFO("data ptr: {}, size: {}", (uint64_t)data, dataSize);

    FrameQueue frameQueue(10);
    frameQueue.start([&](std::shared_ptr<Frame> frame) {
        LOG_INFO("frameQueue callback: frame ptr: {}, size: {}", (uint64_t)frame->getData(), frame->getDataSize());
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    });
    frameQueue.enqueue(depthFrame);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    frameQueue.enqueue(depthFrame);
    frameQueue.enqueue(depthFrame);

    auto queueSize = frameQueue.size();
    LOG_INFO("queue size: {}", queueSize);
    frameQueue.flush();

    queueSize = frameQueue.size();
    LOG_INFO("queue size after flush: {}", queueSize);

    depthFrame.reset();
    depthFrameBufferManager.reset();
    frameMemoryPool.reset();
    FrameMemoryPool::destroyInstance();
    return 0;
}