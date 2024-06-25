#include "NetDeviceEnumerator.hpp"

#include "core/device/mega/FemtoMegaDeviceInfo.hpp"
#include "core/device/mega/FemtoMegaNetDevice.hpp"
#include "core/device/gemini2/Gemini2XLDeviceInfo.hpp"
#include "core/device/gemini2/Gemini2XLDevice.hpp"

#include "private/LicenseManager.hpp"

#include <map>
#include <string>

namespace libobsensor {

const std::map<uint16_t, std::string> pidToNameMap = {
    { 0x0669, "Femto Mega" },
    { 0x06c0, "Femto Mega i" },
    { 0x0671, "Orbbec Gemini2 XL" },
};

NetDeviceEnumerator::NetDeviceEnumerator(std::shared_ptr<ObPal> obPal, DeviceChangedCallback callback)
    : IDeviceEnumerator(obPal), deviceChangedCallback_(callback) {
    // std::unique_lock<std::recursive_mutex> lock(deviceInfoListMutex_); // 在构造函数可以不用对deviceInfoList_加锁
    deviceInfoList_ = queryDeviceList();
    if(!deviceInfoList_.empty()) {
        LOG_DEBUG("Current net device list: ({})", deviceInfoList_.size());
        for(auto &&item: deviceInfoList_) {
            auto info = std::dynamic_pointer_cast<NetSourcePortInfo>(item->sourcePortInfoList_.front());
            LOG_DEBUG("  - Name: {}, PID: 0x{:04X}, SN/ID: {}, MAC:{}, IP:{}", item->name_, item->pid_, item->deviceSn_, info->mac, info->address);
        }
    }
    else {
        LOG_DEBUG("No net device found!");
    }

    deviceWatcher_ = obPal_->createNetDeviceWatcher();
    deviceWatcher_->start([this](OBDeviceChangedType changedType, std::string url) { onPalDeviceChanged(changedType, url); });
}

NetDeviceEnumerator::~NetDeviceEnumerator() {
    deviceWatcher_->stop();
}

std::vector<std::shared_ptr<IDeviceEnumInfo>> NetDeviceEnumerator::queryDeviceList() {
    std::unique_lock<std::recursive_mutex> lock(deviceInfoListMutex_);
    sourcePortInfoList_ = obPal_->queryNetSourcePort();

    if(sourcePortInfoList_.empty()) {
        LOG_DEBUG("No net source port found!");
        return std::vector<std::shared_ptr<IDeviceEnumInfo>>();
    }

    LOG_DEBUG("Current net source port list:");
    for(const auto &item: sourcePortInfoList_) {
        auto info = std::dynamic_pointer_cast<NetSourcePortInfo>(item);
        LOG_DEBUG(" - mac:{}, ip:{}, port:{}", info->mac, info->address, info->port);
    }
    return deviceInfoMatch(sourcePortInfoList_);
}

std::vector<std::shared_ptr<IDeviceEnumInfo>> NetDeviceEnumerator::getDeviceInfoList() {
    std::unique_lock<std::recursive_mutex> lock(deviceInfoListMutex_);
    return deviceInfoList_;
}

std::vector<std::shared_ptr<IDeviceEnumInfo>> NetDeviceEnumerator::deviceInfoMatch(const SourcePortInfoList infoList) {
    std::vector<std::shared_ptr<IDeviceEnumInfo>> deviceInfoList;
    std::map<std::string, SourcePortInfoList>     infoGroups;
    for(auto &&item: infoList) {
        auto info = std::dynamic_pointer_cast<NetSourcePortInfo>(item);
        auto iter = infoGroups.find(info->mac);
        if(iter == infoGroups.end()) {
            iter = infoGroups.insert(iter, { info->mac, SourcePortInfoList() });
        }
        iter->second.push_back(info);
    }

    for(auto &&group: infoGroups) {
        auto                            &item    = group.second.front();
        auto                             info    = std::dynamic_pointer_cast<NetSourcePortInfo>(item);
        std::shared_ptr<IDeviceEnumInfo> devInfo = std::make_shared<IDeviceEnumInfo>();

        auto pidToNameIter = pidToNameMap.find(info->pid);
        devInfo->name_     = pidToNameIter == pidToNameMap.end() ? "Unknown" : pidToNameIter->second;

        devInfo->uid_                = info->mac;
        devInfo->pid_                = info->pid;
        devInfo->vid_                = 0x2BC5;
        devInfo->deviceSn_           = info->serialNumber;
        devInfo->connectionType_     = "Ethernet";
        devInfo->sourcePortInfoList_ = group.second;
        deviceInfoList.emplace_back(devInfo);
    }

    return deviceInfoList;
}

void NetDeviceEnumerator::setDeviceChangedCallback(DeviceChangedCallback callback) {
    std::unique_lock<std::mutex> lock(deviceChangedCallbackMutex_);
    deviceChangedCallback_ = callback;
    // deviceChangedCallback_ = [callback, this](std::vector<std::shared_ptr<IDeviceEnumInfo>> removed, std::vector<std::shared_ptr<IDeviceEnumInfo>> added) {
    //     if(devChangedCallbackThread_.joinable()) {
    //         devChangedCallbackThread_.join();
    //     }
    //     devChangedCallbackThread_ = std::thread(callback, removed, added);
    // };
}

void NetDeviceEnumerator::onPalDeviceChanged(OBDeviceChangedType changeType, std::string devUid) {
    std::vector<std::shared_ptr<IDeviceEnumInfo>> addDevs;
    std::vector<std::shared_ptr<IDeviceEnumInfo>> rmDevs;

    {
        auto                                   devices = queryDeviceList();
        std::unique_lock<std::recursive_mutex> lock(deviceInfoListMutex_);
        addDevs         = ObUtils::subtract_sets(devices, deviceInfoList_);
        rmDevs          = ObUtils::subtract_sets(deviceInfoList_, devices);
        deviceInfoList_ = devices;
    }

    // callback
    std::unique_lock<std::mutex> lock(deviceChangedCallbackMutex_);
    if(deviceChangedCallback_ && (!addDevs.empty() || !rmDevs.empty())) {
        LOG_DEBUG("Net device list changed!");
        if(!addDevs.empty()) {
            LOG_DEBUG("{} net device(s) found:", addDevs.size());
            for(auto &&item: addDevs) {
                auto info = std::dynamic_pointer_cast<NetSourcePortInfo>(item->sourcePortInfoList_.front());
                LOG_DEBUG("  - Name: {}, PID: 0x{:04X}, SN/ID: {}, MAC:{}, IP:{}", item->name_, item->pid_, item->deviceSn_, info->mac, info->address);
            }
        }

        if(!rmDevs.empty()) {
            LOG_DEBUG("{} net device(s) removed:", rmDevs.size());
            for(auto &&item: rmDevs) {
                auto info = std::dynamic_pointer_cast<NetSourcePortInfo>(item->sourcePortInfoList_.front());
                LOG_DEBUG("  - Name: {}, PID: 0x{:04X}, SN/ID: {}, MAC:{}, IP:{}", item->name_, item->pid_, item->deviceSn_, info->mac, info->address);
            }
        }

        LOG_DEBUG("Current net device list: ({})", deviceInfoList_.size());
        for(auto &&item: deviceInfoList_) {
            auto info = std::dynamic_pointer_cast<NetSourcePortInfo>(item->sourcePortInfoList_.front());
            LOG_DEBUG("  - Name: {}, PID: 0x{:04X}, SN/ID: {}, MAC:{}, IP:{}", item->name_, item->pid_, item->deviceSn_, info->mac, info->address);
        }

        deviceChangedCallback_(rmDevs, addDevs);
    }
}

std::shared_ptr<IDevice> NetDeviceEnumerator::createDevice(std::shared_ptr<ObPal> obPal, std::string address, uint16_t port) {
    auto vendorPort = obPal->queryNetVendorPort(address, port);
    if(!vendorPort) {
        throw invalid_value_exception(Obutils::string::to_string() << "Could not find device, address=" << address << ", port=" << port);
    }
    std::shared_ptr<IDevice> device;
    SourcePortInfoList       portInfoList   = { vendorPort };
    auto                     deviceInfoList = deviceInfoMatch(portInfoList);
    auto                     deviceInfo     = deviceInfoList.front();

    auto rstDeviceInfo = associatedSourcePortCompletion(obPal, deviceInfo);

    if(isMatchDeviceByPid(rstDeviceInfo->pid_, gFemtoMegaPids)) {
        device = std::make_shared<FemtoMegaNetDevice>(obPal, rstDeviceInfo);
        LOG_DEBUG("Create Net Device success! address={0}, port={1}, pid=0x{2:4x}", address, port, rstDeviceInfo->pid_);
    }
    else if(isMatchDeviceByPid(rstDeviceInfo->pid_, gGemini2XLPids)) {
        device = std::make_shared<Gemini2XLDevice>(obPal, rstDeviceInfo);
        LOG_DEBUG("Create Net Device success! address={0}, port={1}, pid=0x{2:4x}", address, port, rstDeviceInfo->pid_);
    }
    else {
        LOG_ERROR("Create Net Device failed, unsupported device! address={0}, port={1}, pid=0x{2:04x}", address, port, rstDeviceInfo->pid_);
    }
    return device;
}

std::shared_ptr<IDeviceEnumInfo> NetDeviceEnumerator::associatedSourcePortCompletion(std::shared_ptr<ObPal> obPal, std::shared_ptr<IDeviceEnumInfo> info) {
    auto rstDeviceInfo = std::make_shared<IDeviceEnumInfo>(info);
    if(rstDeviceInfo->sourcePortInfoList_.size() == 1) {
        auto associatedPortList = obPal->queryAssociatedNetSourcePort(rstDeviceInfo->sourcePortInfoList_.front());
        rstDeviceInfo->sourcePortInfoList_.insert(rstDeviceInfo->sourcePortInfoList_.end(), associatedPortList.begin(), associatedPortList.end());
    }
    return rstDeviceInfo;
}

}  // namespace libobsensor