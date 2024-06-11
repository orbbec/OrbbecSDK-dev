#include "Ethernet.hpp"
#include "core/command/protocol/HostProtocol.hpp"
#include "core/command/VendorCommand.hpp"
#include "exception/ObException.hpp"
#include "parameter/Mx6600CalibParamParser.hpp"


namespace libobsensor {

const uint16_t DEFAULT_CMD_PORT                     = 8090;
const uint16_t PID_FEMTO_MEGA                       = 0x0669;
const uint16_t PID_FEMTO_MEGA_I                     = 0x06C0;
const uint16_t PID_GEMINI2XL                        = 0x0671;
const uint16_t DEVICE_WATCHER_POLLING_INTERVAL_MSEC = 5000;

NetDeviceWatcher::~NetDeviceWatcher() noexcept {
    if(!stopWatch_) {
        stop();
    }
};

void NetDeviceWatcher::start(deviceChangedCallback callback) {
    callback_          = callback;
    stopWatch_         = false;
    deviceWatchThread_ = std::thread([&]() {
        std::mutex                   mutex;
        std::unique_lock<std::mutex> lock(mutex);
        while(!stopWatch_) {
            auto list    = GVCPClient::instance().queryNetDeviceList();
            auto added   = utils::subtract_sets(list, netDevInfoList_);
            auto removed = utils::subtract_sets(netDevInfoList_, list);
            for(auto &&info: removed) {
                bool    disconnected = true;
                bool    exception    = false;
                uint8_t retry        = 1;
                if(info.pid == PID_GEMINI2XL) {
                    do {
                        BEGIN_TRY_EXECUTE({
                            auto netVendorPortInfo = std::make_shared<NetSourcePortInfo>(SOURCE_PORT_NET_VENDOR, info.ip, DEFAULT_CMD_PORT);
                            // auto            netVendorPort = std::make_shared<VendorNetDataPort>(netVendorPortInfo, 500, 500);
                            auto            netVendorPort = std::make_shared<VendorNetDataPort>(netVendorPortInfo);
                            auto            hostProtocol  = std::make_shared<HostProtocol>(netVendorPort);
                            auto            command       = std::make_shared<VendorCommand>(hostProtocol);
                            OBPropertyValue pidValue{};
                            command->getPropertyValue(OB_PROP_PID_INT, &pidValue);
                            if(pidValue.intValue != PID_GEMINI2XL) {
                                LOG_WARN("Create socket succeed ip:{} port:{},but pid is invalid {}", info.ip, DEFAULT_CMD_PORT, pidValue.intValue);
                            }
                            netDevInfoList_.push_back(info);
                            disconnected = false;
                        })
                        CATCH_EXCEPTION_AND_EXECUTE({
                            exception = true;
                            LOG_WARN("Create socket failed ip:{} port:{},device is disconnect.", info.ip, DEFAULT_CMD_PORT);
                        })
                    } while(exception && retry-- > 0);
                }
                if(disconnected) {
                    callback_(OB_DEVICE_REMOVED, info.mac);
                }
            }

            for(auto &&info: added) {
                callback_(OB_DEVICE_ARRIVAL, info.mac);
            }

            netDevInfoList_ = list;
            condVar_.wait_for(lock, std::chrono::milliseconds(DEVICE_WATCHER_POLLING_INTERVAL_MSEC), [&]() { return stopWatch_; });
        }
    });
}

void NetDeviceWatcher::stop() {
    stopWatch_ = true;
    condVar_.notify_all();
    if(deviceWatchThread_.joinable()) {
        deviceWatchThread_.join();
    }
}

SourcePortInfoList Ethernet::queryNetSourcePort() {
    std::vector<NetDeviceInfo> infos = GVCPClient::instance().queryNetDeviceList();

    auto added         = utils::subtract_sets(infos, netDeviceInfoList_);
    auto removed       = utils::subtract_sets(netDeviceInfoList_, infos);
    netDeviceInfoList_ = infos;

    // 只对新上线的设备重新查询端口信息
    for(auto &&info: added) {
        if(info.pid == 0) {
            TRY_EXECUTE({
                auto            netVendorPortInfo = std::make_shared<NetSourcePortInfo>(SOURCE_PORT_NET_VENDOR, info.ip, DEFAULT_CMD_PORT);
                auto            netVendorPort     = std::make_shared<VendorNetDataPort>(netVendorPortInfo);
                auto            hostProtocol      = std::make_shared<HostProtocol>(netVendorPort);
                auto            command           = std::make_shared<VendorCommand>(hostProtocol);
                OBPropertyValue pidValue{};
                BEGIN_TRY_EXECUTE({
                    command->getPropertyValue(OB_PROP_PID_INT, &pidValue);
                    info.pid = pidValue.intValue;
                })
                CATCH_EXCEPTION_AND_EXECUTE({ info.pid = PID_FEMTO_MEGA; })
            })
        }

        sourcePortInfoList_.push_back(std::make_shared<NetSourcePortInfo>(SOURCE_PORT_NET_VENDOR, info.ip, DEFAULT_CMD_PORT, info.mac, info.sn, info.pid));
    }

    // 删除列表中已经下线的设备
    for(auto &&info: removed) {
        auto iter = sourcePortInfoList_.begin();
        while(iter != sourcePortInfoList_.end()) {
            auto item = std::dynamic_pointer_cast<NetSourcePortInfo>(*iter);
            if(item->address == info.ip && item->mac == info.mac && item->serialNumber == info.sn) {
                iter = sourcePortInfoList_.erase(iter);
            }
            else {
                ++iter;
            }
        }
    }
    return sourcePortInfoList_;
}

std::shared_ptr<NetSourcePortInfo> Ethernet::queryNetVendorPort(std::string address, uint16_t port) {
    SourcePortInfoList list;
    auto               netVendorPortInfo = std::make_shared<NetSourcePortInfo>(SOURCE_PORT_NET_VENDOR, address, DEFAULT_CMD_PORT);
    auto               netVendorPort     = std::make_shared<VendorNetDataPort>(netVendorPortInfo);
    auto               hostProtocol      = std::make_shared<HostProtocol>(netVendorPort);
    auto               command           = std::make_shared<VendorCommand>(hostProtocol);
    auto               pid               = PID_FEMTO_MEGA;
    TRY_EXECUTE({
        OBPropertyValue pidValue{};
        command->getPropertyValue(OB_PROP_PID_INT, &pidValue);
        pid = pidValue.intValue;
    });

    // 当前所有适配的设备的端口号都为8090（DEFAULT_CMD_PORT），且不可更改
    return std::make_shared<NetSourcePortInfo>(SOURCE_PORT_NET_VENDOR, address, DEFAULT_CMD_PORT, address + ":" + std::to_string(port), "Unknown", pid);
}

bool Ethernet::changeNetDeviceIpConfig(std::string ipAddress, const OBNetIpConfig &config) {
#ifdef _WIN32
    return GVCPClient::instance().changeNetDeviceIpConfig(ipAddress, config);
#else
    return false;
#endif
}

SourcePortInfoList Ethernet::queryAssociatedNetSourcePort(const std::shared_ptr<NetSourcePortInfo> info) {
    SourcePortInfoList list;

    if(info->pid == PID_FEMTO_MEGA || info->pid == PID_FEMTO_MEGA_I) {
        list.emplace_back(std::make_shared<RTSPStreamPortInfo>(info->address, 8888, info->port, OB_STREAM_COLOR, info->mac, info->serialNumber, info->pid));
        list.emplace_back(std::make_shared<RTSPStreamPortInfo>(info->address, 8554, info->port, OB_STREAM_DEPTH, info->mac, info->serialNumber, info->pid));
        list.emplace_back(std::make_shared<RTSPStreamPortInfo>(info->address, 8554, info->port, OB_STREAM_IR, info->mac, info->serialNumber, info->pid));
        list.emplace_back(std::make_shared<NetDataStreamPortInfo>(info->address, 8900, info->port, info->mac, info->serialNumber, info->pid));
    }
    else if(info->pid == PID_GEMINI2XL) {
        list.emplace_back(std::make_shared<RTSPStreamPortInfo>(info->address, 8888, info->port, OB_STREAM_COLOR, info->mac, info->serialNumber, info->pid));
        list.emplace_back(std::make_shared<RTSPStreamPortInfo>(info->address, 8554, info->port, OB_STREAM_DEPTH, info->mac, info->serialNumber, info->pid));
        list.emplace_back(std::make_shared<RTSPStreamPortInfo>(info->address, 8555, info->port, OB_STREAM_IR_LEFT, info->mac, info->serialNumber, info->pid));
        list.emplace_back(std::make_shared<RTSPStreamPortInfo>(info->address, 8556, info->port, OB_STREAM_IR_RIGHT, info->mac, info->serialNumber, info->pid));
        list.emplace_back(std::make_shared<NetDataStreamPortInfo>(info->address, 8900, info->port, info->mac, info->serialNumber, info->pid));
    }
    else {
        auto                                 netVendorPortInfo = std::make_shared<NetSourcePortInfo>(SOURCE_PORT_NET_VENDOR, info->address, info->port);
        auto                                 netVendorPort     = std::make_shared<VendorNetDataPort>(netVendorPortInfo);
        auto                                 hostProtocol      = std::make_shared<HostProtocol>(netVendorPort);
        auto                                 command           = std::make_shared<VendorCommand>(hostProtocol);
        std::vector<OBNetworkStreamPortInfo> networkStreamPortInfos;
        std::vector<uint8_t>                 dataVec;
        TRY_EXECUTE({
            command->getRawData(
                OB_RAW_DATA_NET_PORT_INFO_LIST,
                [&](OBDataTranState state, OBDataChunk *dataChunk) {
                    if(state == DATA_TRAN_STAT_TRANSFERRING) {
                        dataVec.insert(dataVec.end(), dataChunk->data, dataChunk->data + dataChunk->size);
                    }
                },
                false);
            if(!dataVec.empty()) {
                networkStreamPortInfos = processNetworkStreamPortInfoList(dataVec.data(), dataVec.size());
            }
        });
        for(auto streamPortInfo: networkStreamPortInfos) {
            if(streamPortInfo.sourcePortType == SOURCE_PORT_NET_VENDOR_STREAM) {
                list.emplace_back(
                    std::make_shared<NetDataStreamPortInfo>(info->address, streamPortInfo.port, info->port, info->mac, info->serialNumber, info->pid));
            }
            else if(streamPortInfo.sourcePortType == SOURCE_PORT_NET_RTSP) {
                list.emplace_back(std::make_shared<RTSPStreamPortInfo>(info->address, streamPortInfo.port, info->port, streamPortInfo.streamType, info->mac,
                                                                       info->serialNumber, info->pid));
            }
        }
    }
    return list;
}

}  // namespace libobsensor
