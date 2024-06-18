#include "ISourcePort.hpp"
#include <functional>
#include <algorithm>

namespace libobsensor {
namespace utils {

template <typename T> std::vector<std::vector<T>> GroupVector(const std::vector<T> &vec, std::function<bool(const T &elm0, const T &elm1)> compareFunc) {
    std::vector<std::vector<T>> group;
    for(const auto &elm0: vec) {
        auto itVec = group.begin();
        while(itVec != group.end()) {
            if(compareFunc(elm0, itVec->front())) {
                itVec->push_back(elm0);
                break;
            }
            itVec++;
        }
        if(itVec == group.end()) {
            group.push_back(std::vector<T>({ elm0 }));
        }
    }
    return group;
}

#define GroupUSBSourcePortInfo(vec, comp) GroupVector<std::shared_ptr<const SourcePortInfo>>(vec, comp)
bool GroupUSBSourcePortBySN(const std::shared_ptr<const SourcePortInfo>& port0, const std::shared_ptr<const SourcePortInfo>& port1);
bool GroupUSBSourcePortByUrl(const std::shared_ptr<const SourcePortInfo>& port0, const std::shared_ptr<const SourcePortInfo>& port1);

template <typename T> std::vector<std::shared_ptr<const SourcePortInfo>> FilterUSBPortInfoByPid(const std::vector<T> &devInfos, const std::vector<uint16_t> &pids) {
    std::vector<std::shared_ptr<const SourcePortInfo>> outDeviceInfos;
    for(auto &item: devInfos) {
        if(item->portType == SOURCE_PORT_USB_VENDOR || item->portType == SOURCE_PORT_USB_UVC || item->portType == SOURCE_PORT_USB_HID
           || item->portType == SOURCE_PORT_USB_MULTI_UVC) {
            auto dev = std::dynamic_pointer_cast<const USBSourcePortInfo>(item);
            auto iter = std::find (pids.begin(), pids.end(), dev->pid);
            if(iter == pids.end()) {
                continue;
            }
            outDeviceInfos.push_back(item);
        }
    }
    return outDeviceInfos;
}

}  // namespace utils
}  // namespace libobsensor