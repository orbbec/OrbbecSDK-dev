#pragma once
#include "ObPal.hpp"
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <Windows.h>
#include <map>

#include "usb/enumerator/Enumerator.hpp"

namespace libobsensor {


class WinPal : public std::enable_shared_from_this<WinPal>, virtual public ObPal {
private:
    WinPal();

    static std::weak_ptr<WinPal> instanceWeakPtr_;
    static std::mutex instanceMutex_;
    friend std::shared_ptr<ObPal> ObPal::getInstance();

public:
    virtual ~WinPal() noexcept override;

    std::shared_ptr<ISourcePort>    createSourcePort(std::shared_ptr<const SourcePortInfo> portInfo) override;
    std::shared_ptr<DeviceWatcher> createUsbDeviceWatcher() const override;
    SourcePortInfoList             queryUsbSourcePort() override;

private:
    std::shared_ptr<UsbEnumerator> usbEnumerator_;

private:
    std::mutex                                                                 sourcePortMapMutex_;
    std::map<std::shared_ptr<const SourcePortInfo>, std::weak_ptr<ISourcePort>> sourcePortMap_;
};

class WinUsbDeviceWatcher : public DeviceWatcher {
public:
     WinUsbDeviceWatcher(const ObPal *backend);
    ~WinUsbDeviceWatcher() noexcept override;

    void start(deviceChangedCallback callback) override;
    void stop() override;

private:
    void                    run();
    static LRESULT CALLBACK onWinEvent(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
    static bool             registerDeviceInterfaceToHwnd(HWND hWnd);

private:
    std::thread eventThread_;
    std::mutex  mutex_;
    struct extra_data {
        const ObPal          *backend_;
        deviceChangedCallback callback_;
        bool                  stopped_;
        HWND                  hWnd;
        HDEVNOTIFY            hDevNotifyHW;
        HDEVNOTIFY            hDevNotifyUVC;
        HDEVNOTIFY            hDevNotifySensor;
        HDEVNOTIFY            hDevNotifyHID;
        HDEVNOTIFY            hDevNotifyUSB;
        HDEVNOTIFY            hDevNotifyOpenNI;
    } extraData_;
};


}  // namespace libobsensor
