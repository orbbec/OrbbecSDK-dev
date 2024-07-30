#include "FemtoMegaDeviceInfo.hpp"
#include "FemtoMegaUvcDevice.hpp"
#include "usb/UsbGroup.hpp"
#include "DevicePids.hpp"


namespace libobsensor {
FemtoMegaDeviceInfo::FemtoMegaDeviceInfo(const SourcePortInfoList groupedInfoList) {
    auto portInfo = std::dynamic_pointer_cast<const USBSourcePortInfo>(groupedInfoList.front());

    name_               = "FemtoMega";
    pid_                = portInfo->pid;
    vid_                = portInfo->vid;
    uid_                = portInfo->uid;
    deviceSn_           = portInfo->serial;
    connectionType_     = portInfo->connSpec;
    sourcePortInfoList_ = groupedInfoList;
}

FemtoMegaDeviceInfo::~FemtoMegaDeviceInfo() noexcept {}

std::shared_ptr<IDevice> FemtoMegaDeviceInfo::createDevice() const {
    auto device = std::make_shared<FemtoMegaUvcDevice>(shared_from_this());
    return device;
}

std::vector<std::shared_ptr<IDeviceEnumInfo>> FemtoMegaDeviceInfo::createDeviceInfos(const SourcePortInfoList infoList) {
    std::vector<std::shared_ptr<IDeviceEnumInfo>> femtoMegaDeviceInfos;
    auto                                          remainder = FilterUSBPortInfoByPid(infoList, FemtoMegaDevPids);
    auto                                          groups    = GroupUSBSourcePortInfo(remainder, GroupUSBSourcePortByUrl);
    auto                                          iter      = groups.begin();
    while(iter != groups.end()) {
        if(iter->size() >= 2) {
            auto info = std::make_shared<FemtoMegaDeviceInfo>(*iter);
            femtoMegaDeviceInfos.push_back(info);
        }
        iter++;
    }
    return femtoMegaDeviceInfos;
}
}  // namespace libobsensor