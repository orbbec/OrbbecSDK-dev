#include "FemtoBoltDeviceInfo.hpp"
#include "FemtoBoltDevice.hpp"
#include "utils/UsbGroup.hpp"
#include "DevicePids.hpp"


namespace libobsensor {
FemtoBoltDeviceInfo::FemtoBoltDeviceInfo(const SourcePortInfoList groupedInfoList) {
    auto portInfo = std::dynamic_pointer_cast<const USBSourcePortInfo>(groupedInfoList.front());

    name_               = "FemtoBolt";
    pid_                = portInfo->pid;
    vid_                = portInfo->vid;
    uid_                = portInfo->uid;
    deviceSn_           = portInfo->serial;
    connectionType_     = portInfo->connSpec;
    sourcePortInfoList_ = groupedInfoList;
}

FemtoBoltDeviceInfo::~FemtoBoltDeviceInfo() noexcept {}

std::shared_ptr<IDevice> FemtoBoltDeviceInfo::createDevice() const {
    auto device = std::make_shared<FemtoBoltDevice>(shared_from_this());
    return device;
}

std::vector<std::shared_ptr<IDeviceEnumInfo>> FemtoBoltDeviceInfo::createDeviceInfos(const SourcePortInfoList infoList) {
    std::vector<std::shared_ptr<IDeviceEnumInfo>> femtoBoltDeviceInfos;
    auto                                          remainder = utils::FilterUSBPortInfoByPid(infoList, FemtoBoltDevPids);
    auto                                          groups    = utils::GroupUSBSourcePortInfo(remainder, utils::GroupUSBSourcePortByUrl);
    auto                                          iter      = groups.begin();
    while(iter != groups.end()) {
        if(iter->size() >= 2) {
            auto info = std::make_shared<FemtoBoltDeviceInfo>(*iter);
            femtoBoltDeviceInfos.push_back(info);
        }
        iter++;
    }
    return femtoBoltDeviceInfos;
}
}  // namespace libobsensor