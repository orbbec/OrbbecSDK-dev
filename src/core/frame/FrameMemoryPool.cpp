#include "FrameMemoryPool.hpp"
#include "CoreTypeHelper.hpp"
#include <sstream>

namespace ob {
namespace core {

std::mutex                       instanceMutex;
std::shared_ptr<FrameMemoryPool> instance;
bool                             reuseFrameBufferManager = true;

std::shared_ptr<FrameMemoryPool> FrameMemoryPool::getInstance() {
    std::unique_lock<std::mutex> lk(instanceMutex);
    if(!instance) {
        instance = std::shared_ptr<FrameMemoryPool>(new FrameMemoryPool());
    }
    return instance;
}

void FrameMemoryPool::releaseInstance() {
    std::unique_lock<std::mutex> lk(instanceMutex);
    instance.reset();
}

void FrameMemoryPool::setMaxFrameMemorySize(uint64_t sizeInMB) {
    FrameMemoryAllocator::getInstance()->setMaxFrameMemorySize(sizeInMB);
}

void FrameMemoryPool::activateFrameBufferManagerReuse(bool enable) {
    reuseFrameBufferManager = enable;
}

FrameMemoryPool::FrameMemoryPool() {
    LOG_DEBUG("FrameMemoryPool created!");
}

FrameMemoryPool::~FrameMemoryPool() {
    bufMgrMap_.clear();
}

std::shared_ptr<IFrameBufferManager> FrameMemoryPool::createFrameBufferManager(OBFrameType type, uint32_t maxDataSize) {
    std::unique_lock<std::mutex> lock(bufMgrMapMutex_);
    FrameBufferManagerInfo       info = { type, maxDataSize };

    if(reuseFrameBufferManager) {
        auto iter = bufMgrMap_.find(info);
        if(iter != bufMgrMap_.end()) {
            return iter->second;
        }
    }

    std::shared_ptr<IFrameBufferManager> frameBufMgr;
    switch(type) {
    case OB_FRAME_DEPTH:
        frameBufMgr = std::shared_ptr<FrameBufferManager<DepthFrame>>(new FrameBufferManager<DepthFrame>(maxDataSize));
        LOG_DEBUG("DepthFrame bufferManager created!");
        break;
    case OB_FRAME_IR_LEFT:
        frameBufMgr = std::shared_ptr<FrameBufferManager<IRLeftFrame>>(new FrameBufferManager<IRLeftFrame>(maxDataSize));
        LOG_DEBUG("IRFrame bufferManager created!");
        break;
    case OB_FRAME_IR_RIGHT:
        frameBufMgr = std::shared_ptr<FrameBufferManager<IRRightFrame>>(new FrameBufferManager<IRRightFrame>(maxDataSize));
        LOG_DEBUG("IRFrame bufferManager created!");
        break;
    case OB_FRAME_IR:
        frameBufMgr = std::shared_ptr<FrameBufferManager<IRFrame>>(new FrameBufferManager<IRFrame>(maxDataSize));
        LOG_DEBUG("IRFrame bufferManager created!");
        break;
    case OB_FRAME_COLOR:
        frameBufMgr = std::shared_ptr<FrameBufferManager<ColorFrame>>(new FrameBufferManager<ColorFrame>(maxDataSize));
        LOG_DEBUG("ColorFrame bufferManager created!");
        break;
    case OB_FRAME_GYRO:
        frameBufMgr = std::shared_ptr<FrameBufferManager<GyroFrame>>(new FrameBufferManager<GyroFrame>(maxDataSize));
        LOG_DEBUG("GyroFrame bufferManager created!");
        break;
    case OB_FRAME_ACCEL:
        frameBufMgr = std::shared_ptr<FrameBufferManager<AccelFrame>>(new FrameBufferManager<AccelFrame>(maxDataSize));
        LOG_DEBUG("AccelFrame bufferManager created!");
        break;
    case OB_FRAME_POINTS:
        frameBufMgr = std::shared_ptr<FrameBufferManager<PointsFrame>>(new FrameBufferManager<PointsFrame>(maxDataSize));
        LOG_DEBUG("PointsFrame bufferManager created!");
        break;
    case OB_FRAME_SET:
        frameBufMgr = std::shared_ptr<FrameBufferManager<FrameSet>>(new FrameBufferManager<FrameSet>(maxDataSize));
        LOG_DEBUG("Frameset bufferManager created!");
        break;
    default:
        std::ostringstream oss_msg;
        oss_msg << "Unsupported Frame Type to create buffer manager! frameType: " << type;
        throw unsupported_operation_exception(oss_msg.str());
        break;
    }

    if(reuseFrameBufferManager) {
        bufMgrMap_.insert({ info, frameBufMgr });
    }
    else {
        auto iter = bufMgrWeakList_.begin();
        while(iter != bufMgrWeakList_.end()) {
            if(iter->expired()) {
                iter = bufMgrWeakList_.erase(iter);
                continue;
            }
            iter++;
        }
        bufMgrWeakList_.push_back(frameBufMgr);
    }

    return frameBufMgr;
}

std::shared_ptr<IFrameBufferManager> FrameMemoryPool::createFrameBufferManager(OBFrameType type, std::shared_ptr<const StreamProfile> streamProfile) {
    if(streamProfile->is<VideoStreamProfile>()) {
        auto sp = streamProfile->as<VideoStreamProfile>();
        return createFrameBufferManager(type, sp->getFormat(), sp->getWidth(), sp->getHeight());
    }
    else if(streamProfile->is<AccelStreamProfile>()) {
        return createFrameBufferManager(type, sizeof(AccelFrame::OBAccelFrameData));
    }
    else if(streamProfile->is<GyroStreamProfile>()) {
        return createFrameBufferManager(type, sizeof(GyroFrame::OBGyroFrameData));
    }
    LOG_WARN("unsupported streamProfile type");
    return nullptr;
}

std::shared_ptr<IFrameBufferManager> FrameMemoryPool::createFrameBufferManager(OBFrameType type, OBFormat format, uint32_t width, uint32_t height) {
    auto maxDataSize = type_helper::calcVideoFrameMaxDataSize(format, width, height);
    return createFrameBufferManager(type, maxDataSize);
}

void FrameMemoryPool::freeIdleMemory() {
    std::unique_lock<std::mutex> lock(bufMgrMapMutex_);
    auto                         iter = bufMgrMap_.begin();
    while(iter != bufMgrMap_.end()) {
        if(iter->second.use_count() == 1) {
            iter = bufMgrMap_.erase(iter);
            continue;
        }
        iter->second->releaseIdleBuffer();
        iter++;
    }

    auto vecIter = bufMgrWeakList_.begin();
    while(vecIter != bufMgrWeakList_.end()) {
        if(vecIter->expired()) {
            vecIter = bufMgrWeakList_.erase(vecIter);
            continue;
        }
        auto bufMgr = vecIter->lock();
        if(bufMgr) {
            bufMgr->releaseIdleBuffer();
        }
        vecIter++;
    }
}

}  // namespace core
}  // namespace ob