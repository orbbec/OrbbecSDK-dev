#include "G330DeviceInfo.hpp"
#include "G330Device.hpp"
#include "utils/UsbGroup.hpp"
#include "DevicePids.hpp"

#include <map>

namespace libobsensor {


const std::map<int, std::string> G300DeviceNameMap = {
    { 0x06d0, "Orbbec Gemini 2R" },     { 0x06d1, "Orbbec Gemini 2RL" },   { 0x0800, "Orbbec Gemini 335" },   { 0x0801, "Orbbec Gemini 330" },
    { 0x0802, "Orbbec Gemini dm330" },  { 0x0803, "Orbbec Gemini 336" },   { 0x0804, "Orbbec Gemini 335L" },  { 0x0805, "Orbbec Gemini 330L" },
    { 0x0806, "Orbbec Gemini dm330L" }, { 0x0807, "Orbbec Gemini 336L" },  { 0x080B, "Orbbec Gemini 335Lg" }, { 0x080C, "Orbbec Gemini 330Lg" },
    { 0x080D, "Orbbec Gemini 336Lg" },  { 0x080E, "Orbbec Gemini 335Le" }, { 0x080F, "Orbbec Gemini 330Le" }, { 0x0810, "Orbbec Gemini 336Le" },
};

G330DeviceInfo::G330DeviceInfo(const SourcePortInfoList groupedInfoList) {
    auto portInfo = std::dynamic_pointer_cast<const USBSourcePortInfo>(groupedInfoList.front());

    auto iter = G300DeviceNameMap.find(portInfo->pid);
    if(iter != G300DeviceNameMap.end()) {
        name_ = iter->second;
    }
    else {
        name_ = "Orbbec Gemini 300 series device";
    }

    pid_            = portInfo->pid;
    vid_            = portInfo->vid;
    uid_            = portInfo->uid;
    deviceSn_       = portInfo->serial;
    connectionType_ = portInfo->connSpec;
    sourcePortInfoList_ = groupedInfoList;
}

G330DeviceInfo::~G330DeviceInfo() noexcept {}

std::shared_ptr<IDevice> G330DeviceInfo::createDevice() const{
    auto device = std::make_shared<G330Device>(shared_from_this());
    return device;
}

std::vector<std::shared_ptr<IDeviceEnumInfo>> G330DeviceInfo::createDeviceInfos(const SourcePortInfoList infoList) {
    std::vector<std::shared_ptr<IDeviceEnumInfo>> G330DeviceInfos;
    auto remainder = utils::FilterUSBPortInfoByPid(infoList, gG330Pids);
    auto groups    = utils::GroupUSBSourcePortInfo(remainder, utils::GroupUSBSourcePortByUrl);
    auto iter      = groups.begin();
    while(iter != groups.end()) {
        if(iter->size() >= 3) {
            auto info = std::make_shared<G330DeviceInfo>( *iter);
            G330DeviceInfos.push_back(info);
        }
        iter++;
    }

    return G330DeviceInfos;
}

}  // namespace libobsensor
