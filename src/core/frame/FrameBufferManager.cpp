#include "FrameBufferManager.hpp"
#include "FrameMemoryPool.hpp"

#include "exception/ObException.hpp"
#include "logger/Logger.hpp"

namespace libobsensor{


#define DEFAULT_MAX_FRAME_MEMORY_SIZE ((uint64_t)2 * 1024 * 1024 * 1024)  // 2GB

FrameMemoryAllocator::FrameMemoryAllocator() : maxSize_(DEFAULT_MAX_FRAME_MEMORY_SIZE), usedSize_(0) {}
FrameMemoryAllocator::~FrameMemoryAllocator() noexcept {
    if(usedSize_ > 0) {
        LOG_WARN("FrameMemoryAllocator destroyed while still has memory used! usedSize={0:.3f}MB", byteToMB(usedSize_));
    }
}

std::shared_ptr<FrameMemoryAllocator> FrameMemoryAllocator::getInstance() {
    static std::shared_ptr<FrameMemoryAllocator> frameMemoryAllocator = std::shared_ptr<FrameMemoryAllocator>(new FrameMemoryAllocator());
    return frameMemoryAllocator;
}

void FrameMemoryAllocator::setMaxFrameMemorySize(uint64_t sizeInMb) {
    std::unique_lock<std::mutex> lock(mutex_);
    maxSize_ = sizeInMb * 1024 * 1024;
    if(maxSize_ < usedSize_) {
        LOG_WARN("The max frame memory size you set is {:.3f}MB,  less than the current used size, will set to {:.3f}MB instead", byteToMB(maxSize_),
                 byteToMB(usedSize_));
    }
    if(sizeInMb < 100) {  // 100 MB
        LOG_WARN("The size you is less than 100MB, size={:.3f}MB, will set to 100MB instead", (double)sizeInMb);
        maxSize_ = 100 * 1024 * 1024;
    }
}

uint8_t *FrameMemoryAllocator::allocate(size_t size) {
    std::unique_lock<std::mutex> lock(mutex_);
    if(usedSize_ + size > maxSize_) {
        LOG_WARN("FrameMemoryAllocator out of memory! require={0:.3f}MB, total usage: allocated={1:.3f}MB, max limit={2:.3f}MB", byteToMB(size),
                 byteToMB(usedSize_), byteToMB(maxSize_));
        return nullptr;
    }

    void *ptr = malloc(size);
    if(ptr == nullptr && reinterpret_cast<uintptr_t>(ptr) == 0xdddddddd) {
        LOG_ERROR("FrameMemoryAllocator malloc failed! ptr={0:x}", (uintptr_t)ptr);
    }

    memset(ptr, 0, size);
    usedSize_ += size;
    LOG_DEBUG("New frame buffer allocated={0:.3f}MB, total usage: allocated={1:.3f}MB, max limit={2:.3f}MB", byteToMB(size), byteToMB(usedSize_),
              byteToMB(maxSize_));
    return (uint8_t *)ptr;
}

void FrameMemoryAllocator::deallocate(uint8_t *ptr, size_t size) {
    std::unique_lock<std::mutex> lock(mutex_);
    usedSize_ -= size;
    free(ptr);
    LOG_DEBUG("Frame buffer released={0:.3f}MB, total usage: allocated={1:.3f}MB, max limit={2:.3f}MB", byteToMB(size), byteToMB(usedSize_),
              byteToMB(maxSize_));
}

FrameBufferManagerBase::FrameBufferManagerBase(size_t frameDataBufferSize, size_t frameObjSize)
    : frameDataBufferSize_(frameDataBufferSize), frameObjSize_(frameObjSize) {
    frameTotalSize_ = frameDataBufferSize_ + frameObjSize_ + FRAME_DATA_ALIGN_IN_BYTE - 1;  // 多申请FRAME_DATA_ALIGN_IN_BYTE-1, 方便偏移数据部分地址，实现对齐
}

FrameBufferManagerBase::~FrameBufferManagerBase() noexcept {
    std::unique_lock<std::recursive_mutex> lock_(mutex_);
    while(!availableFrameBuffers_.empty()) {
        FrameMemoryAllocator::getInstance()->deallocate(availableFrameBuffers_.front(), frameTotalSize_);
        availableFrameBuffers_.erase(availableFrameBuffers_.begin());
    }
    LOG_DEBUG("FrameBufferManagerBase destroyed! manager type:{0},  obj addr:0x{1:x}", typeid(*this).name(), uint64_t(this));
}

uint8_t *FrameBufferManagerBase::acquireBuffer() {
    std::unique_lock<std::recursive_mutex> lock_(mutex_);
    uint8_t                               *bufferPtr = nullptr;
    if(!availableFrameBuffers_.empty()) {
        bufferPtr = *availableFrameBuffers_.begin();
        availableFrameBuffers_.erase(availableFrameBuffers_.begin());
    }
    else {
        bufferPtr = FrameMemoryAllocator::getInstance()->allocate(frameTotalSize_);
        if(bufferPtr == nullptr) {
            LOG_WARN("allocBuffer failed! Will retry after release idle memory on FrameMemoryPool");
            auto memoryPool = FrameMemoryPool::getInstance();
            memoryPool->freeIdleMemory();
            bufferPtr = FrameMemoryAllocator::getInstance()->allocate(frameTotalSize_);
            if(bufferPtr == nullptr) {
                auto msg = std::string("Alloc frame buffer failed! size=") + std::to_string(frameTotalSize_);
                LOG_FATAL(msg);
                throw memory_exception(msg);
            }
        }
    }
    return bufferPtr;
}

std::unique_lock<std::recursive_mutex> FrameBufferManagerBase::lockBuffers() {
    std::unique_lock<std::recursive_mutex> lock_(mutex_);
    return std::move(lock_);
}

void FrameBufferManagerBase::reclaimBuffer(void *buffer) {
    std::unique_lock<std::recursive_mutex> lock_(mutex_);
    availableFrameBuffers_.push_back((uint8_t *)buffer);

    if(availableFrameBuffers_.size() > 100) {
        // 当availableFrameBuffers_ 足够多时及时及时释放内存
        // 不能直接删除当前buffer，会导致frame析构崩溃
        FrameMemoryAllocator::getInstance()->deallocate(availableFrameBuffers_.front(), frameTotalSize_);
        availableFrameBuffers_.erase(availableFrameBuffers_.begin());
    }
}
void FrameBufferManagerBase::releaseIdleBuffer() {
    std::unique_lock<std::recursive_mutex> lock_(mutex_);
    while(!availableFrameBuffers_.empty()) {
        // 当availableFrameBuffers_ 足够多时及时及时释放内存
        // 不能直接删除当前buffer，会导致frame析构崩溃
        FrameMemoryAllocator::getInstance()->deallocate(availableFrameBuffers_.front(), frameTotalSize_);
        availableFrameBuffers_.erase(availableFrameBuffers_.begin());
    }
}


}  // namespace ob