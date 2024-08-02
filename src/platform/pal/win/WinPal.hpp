#pragma once
#include "ObPal.hpp"
#include <map>

#include "usb/enumerator/IUsbEnumerator.hpp"

namespace libobsensor {

class WinPal : virtual public ObPal, public std::enable_shared_from_this<WinPal> {
private:
    WinPal();

    static std::weak_ptr<WinPal> instanceWeakPtr_;
    static std::mutex instanceMutex_;
    friend std::shared_ptr<ObPal> ObPal::getInstance();

public:
    virtual ~WinPal() noexcept override;

    std::shared_ptr<ISourcePort>   getSourcePort(std::shared_ptr<const SourcePortInfo> portInfo) override;
    std::shared_ptr<DeviceWatcher> createUsbDeviceWatcher() const override;
    SourcePortInfoList             queryUsbSourcePortInfos() override;

private:
    std::shared_ptr<IUsbEnumerator> usbEnumerator_;

private:
    std::mutex                                                                 sourcePortMapMutex_;
    std::map<std::shared_ptr<const SourcePortInfo>, std::weak_ptr<ISourcePort>> sourcePortMap_;
};

}  // namespace libobsensor
