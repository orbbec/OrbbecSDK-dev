#include "NetDataStreamPort.hpp"
#include "logger/Logger.hpp"
#include "exception/ObException.hpp"

namespace libobsensor {

NetDataStreamPort::NetDataStreamPort(std::shared_ptr<const NetDataStreamPortInfo> portInfo) : isStreaming_(false), portInfo_(portInfo) {}

NetDataStreamPort::~NetDataStreamPort() {
    isStreaming_ = false;
    if(readDataThread_.joinable()) {
        readDataThread_.join();
    }

    if(tcpClient_) {
        tcpClient_.reset();
    }
}

void NetDataStreamPort::removeWatcher(std::weak_ptr<DataStreamWatcher> watcher) {
    LOG_DEBUG("NetDataStreamPort::removeWatcher start");
    bool watchersEmpty = false;
    {
        std::unique_lock<std::mutex> lk(watchersMutex_);
        auto                         iter = watchers_.find(watcher);
        if(iter != watchers_.end()) {
            watchers_.erase(iter);
        }
        watchersEmpty = watchers_.empty();
    }

    if(watchersEmpty) {
        LOG_DEBUG("NetDataStreamPort::removeWatcher reset client");
        isStreaming_ = false;
        if(tcpClient_) {
            tcpClient_->flush();
        }
        if(readDataThread_.joinable()) {
            readDataThread_.join();
        }
        std::unique_lock<std::mutex> lk(clientMutex_);
        if(tcpClient_) {
            tcpClient_.reset();
        }
    }

    LOG_DEBUG("NetDataStreamPort::removeWatcher done");
}

void NetDataStreamPort::addWatcher(std::weak_ptr<DataStreamWatcher> watcher) {
    LOG_DEBUG("NetDataStreamPort::addWatcher start");
    {
        std::unique_lock<std::mutex> lk(watchersMutex_);
        watchers_.insert(watcher);
    }
    if(isStreaming_ == false) {
        std::unique_lock<std::mutex> lk(clientMutex_);
        isStreaming_ = true;
        LOG_DEBUG("NetDataStreamPort::addWatcher create client");
        auto noConstPortInfo = std::const_pointer_cast<NetDataStreamPortInfo>(portInfo_);
        tcpClient_ = std::make_shared<VendorTCPClient>(noConstPortInfo->address, noConstPortInfo->port);
        readDataThread_ = std::thread(&NetDataStreamPort::readData, this);
    }
    LOG_DEBUG("NetDataStreamPort::addWatcher done");
}

void NetDataStreamPort::readData() {
    const int                IMU_PACKS_SIZE = 248;
    std::shared_ptr<uint8_t> buffPtr(new uint8_t[IMU_PACKS_SIZE], std::default_delete<uint8_t[]>());
    uint8_t                 *data         = buffPtr.get();
    int                      dataFillSize = 0;
    int                      readSize     = 0;
    while(isStreaming_) {
        {
            std::unique_lock<std::mutex> lk(clientMutex_);
            BEGIN_TRY_EXECUTE({ readSize = tcpClient_->read(data + dataFillSize, IMU_PACKS_SIZE - dataFillSize); })
            CATCH_EXCEPTION_AND_EXECUTE({
                LOG_WARN("read data failed!");
                readSize = -1;
            })
        }

        if(readSize < 0) {
            dataFillSize = 0;
            memset(data, 0, IMU_PACKS_SIZE);
        }
        else {
            dataFillSize += readSize;
        }

        if(IMU_PACKS_SIZE == dataFillSize && isStreaming_) {
            std::set<std::weak_ptr<DataStreamWatcher>, dataStreamWatcherWeakPtrCompare> tempWatchers;

            {
                // 避免对watchers_加锁锁住item->onDataReceived的调用过程
                std::unique_lock<std::mutex> lk(watchersMutex_);
                tempWatchers = watchers_;
            }

            for(auto &watcher: tempWatchers) {
                auto item = watcher.lock();
                if(item) {
                    item->onDataReceived(data, IMU_PACKS_SIZE);
                }
            }

            dataFillSize = 0;
            memset(data, 0, IMU_PACKS_SIZE);
        }
    }
}

}  // namespace libobsensor