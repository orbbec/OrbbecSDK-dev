#pragma once

#include <memory>
#include <functional>
#include <string>
#include "FrameBufferManager.hpp"

namespace libobsensor {

struct FrameBufferManagerInfo {
    OBFrameType frameType;
    size_t      maxFrameDataSize;
};

struct FrameBufferManagerInfoCompare {
    bool operator()(const FrameBufferManagerInfo &l, const FrameBufferManagerInfo &r) const {
        if(l.frameType == r.frameType) {
            return l.maxFrameDataSize < r.maxFrameDataSize;
        }
        return l.frameType < r.frameType;
    }
};

class FrameMemoryPool {
public:
    ~FrameMemoryPool();
    static std::shared_ptr<FrameMemoryPool> getInstance();
    static void                             destroyInstance();
    static void                             setMaxFrameMemorySize(uint64_t sizeInMB);
    static void                             activateFrameBufferManagerReuse(bool enable);

    std::shared_ptr<IFrameBufferManager> createFrameBufferManager(OBFrameType type, size_t frameBufferSize);
    std::shared_ptr<IFrameBufferManager> createFrameBufferManager(OBFrameType type, std::shared_ptr<const StreamProfile> streamProfile);
    std::shared_ptr<IFrameBufferManager> createFrameBufferManager(OBFrameType type, OBFormat format, uint32_t width, uint32_t height);

    void freeIdleMemory();

private:
    FrameMemoryPool();

private:
    std::map<FrameBufferManagerInfo, std::shared_ptr<IFrameBufferManager>, FrameBufferManagerInfoCompare> bufMgrMap_;
    std::mutex                                                                                            bufMgrMapMutex_;
    std::vector<std::weak_ptr<IFrameBufferManager>>                                                       bufMgrWeakList_;
};

}  // namespace libobsensor
