#include "UsbDeviceEnumerator.hpp"
#include "utils/Utils.hpp"

namespace libobsensor {
<<<<<<< HEAD
UsbDeviceEnumerator::UsbDeviceEnumerator(DeviceChangedCallback callback) : obPal_(pal::ObPal::getInstance()) {
    devChangedCallback_ = [callback, this](const std::vector<std::shared_ptr<DeviceEnumInfo>> &removedList,
                                           const std::vector<std::shared_ptr<DeviceEnumInfo>> &addedList) {
        (void)this;
=======
UsbDeviceEnumerator::UsbDeviceEnumerator( DeviceChangedCallback callback) : obPal_(pal::ObPal::getInstance()) {
    devChangedCallback_ = [callback, this](const DeviceEnumInfoList& removedList, const DeviceEnumInfoList& addedList) {
>>>>>>> 2b5680b (Implementing Context and Device Manager.)
#ifdef __ANDROID__
        // 在安卓平台需要在同线程内回调到java，并在回调函数内完成相关资源释放
        callback(removedList, addedList);
#elif defined(__linux__)
        // 解决linux短时间内多次回调，并且用户回调函数内调用有访问libusb的相关接口导致死锁问题。
        auto cbThread = std::thread(callback, removedList, addedList);
        cbThread.detach();
#else
        // WIN 平台由于回调是MF相关线程调用，如果不切换线程直接回调到用户程序，用户程序需要更新MFC界面，会导致程序崩溃；
        if(devChangedCallbackThread_.joinable()) {
            devChangedCallbackThread_.join();
        }
        devChangedCallbackThread_ = std::thread(callback, removedList, addedList);
#endif
    };

    deviceInfoList_ = queryArrivalDevice();

    deviceArrivalHandleThread_ = std::thread(&UsbDeviceEnumerator::deviceArrivalHandleThreadFunc, this);

    deviceWatcher_ = obPal_->createUsbDeviceWatcher();
    deviceWatcher_->start([this](pal::OBDeviceChangedType changedType, std::string url) { onPalDeviceChanged(changedType, url); });

    std::unique_lock<std::recursive_mutex> lock(deviceInfoListMutex_);
    if(!deviceInfoList_.empty()) {
        LOG_DEBUG("Found {} device(s):", deviceInfoList_.size());
        for(auto &deviceInfo: deviceInfoList_) {
            LOG_DEBUG("  - Name: {}, PID: 0x{:04X}, SN/ID: {}, connection: {}", deviceInfo->name_, deviceInfo->pid_, deviceInfo->deviceSn_,
                      deviceInfo->connectionType_);
        }
    }
    else {
        LOG_DEBUG("No matched usb device found!");
    }
}

UsbDeviceEnumerator::~UsbDeviceEnumerator() noexcept {
    destroy_ = true;
    obPal_   = nullptr;

    newUsbPortArrivalCV_.notify_all();
    if(deviceArrivalHandleThread_.joinable()) {
        deviceArrivalHandleThread_.join();
    }

    if(devChangedCallbackThread_.joinable()) {
        devChangedCallbackThread_.join();
    }
}

void UsbDeviceEnumerator::onPalDeviceChanged(pal::OBDeviceChangedType changeType, std::string devUid) {
    if(changeType == pal::OB_DEVICE_REMOVED) {
        std::vector<std::shared_ptr<const DeviceEnumInfo>> removedDevList;
        {
            std::unique_lock<std::recursive_mutex> lock(deviceInfoListMutex_);
            removedDevList  = queryRemovedDevice(devUid);
            deviceInfoList_ = utils::subtract_sets(deviceInfoList_, removedDevList);
        }
        if(removedDevList.size()) {
            LOG_DEBUG("device list changed: removed={0}, current={1}", removedDevList.size(), deviceInfoList_.size());
            if(!removedDevList.empty()) {
                LOG_DEBUG("Removed device list:");
                for(auto &deviceInfo: removedDevList) {
                    LOG_DEBUG("  - Name: {}, PID: 0x{:04X}, SN/ID: {}", deviceInfo->name_, deviceInfo->pid_, deviceInfo->deviceSn_);
                }
            }
            if(!deviceInfoList_.empty()) {
                LOG_DEBUG("Remained device list:");
                for(auto &deviceInfo: deviceInfoList_) {
                    LOG_DEBUG("  - Name: {}, PID: 0x{:04X}, SN/ID: {}", deviceInfo->name_, deviceInfo->pid_, deviceInfo->deviceSn_);
                }
            }
            std::unique_lock<std::mutex> lock(callbackMutex_);
            if(!destroy_ && devChangedCallback_) {
                devChangedCallback_(removedDevList, {});
            }
        }
    }
    else {  // OB_DEVICE_ARRIVAL
        newUsbPortArrival_ = true;
        newUsbPortArrivalCV_.notify_all();
    }
}

DeviceEnumInfoList UsbDeviceEnumerator::queryRemovedDevice(std::string rmDevUid) {
    auto portInfoList = currentUsbPortInfoList_;
    auto iter         = portInfoList.begin();
    while(iter != portInfoList.end()) {
        auto portInfo = std::dynamic_pointer_cast<const USBSourcePortInfo>(*iter);
        if(portInfo->url == rmDevUid || portInfo->infUrl == rmDevUid) {
            iter = portInfoList.erase(iter);
            LOG_DEBUG("usb device removed: {}", rmDevUid);
            continue;
        }
        iter++;
    }
    LOG_DEBUG("Current usb device port list:");
    for(const auto &item: portInfoList) {
        auto portInfo = std::dynamic_pointer_cast<const USBSourcePortInfo>(item);
        LOG_DEBUG(" - {0} | {1}", portInfo->infUrl, portInfo->infName);
    }

    std::unique_lock<std::recursive_mutex> lock(deviceInfoListMutex_);
    if(portInfoList != currentUsbPortInfoList_) {
        currentUsbPortInfoList_                          = portInfoList;
        DeviceEnumInfoList curList = usbDeviceInfoMatch(portInfoList);
        return utils::subtract_sets(deviceInfoList_, curList);
    }
    return {};
}

DeviceEnumInfoList UsbDeviceEnumerator::queryArrivalDevice() {
    std::unique_lock<std::recursive_mutex> lock(deviceInfoListMutex_);
#if defined(BUILD_USB_PORT)
    auto portInfoList = obPal_->queryUsbSourcePort();
    if(portInfoList != currentUsbPortInfoList_) {
        currentUsbPortInfoList_ = portInfoList;
        LOG_DEBUG("Current usb device port list:");
        for(const auto &item: currentUsbPortInfoList_) {
            auto portInfo = std::dynamic_pointer_cast<const USBSourcePortInfo>(item);
            LOG_DEBUG(" - {0} | {1}", portInfo->infUrl, portInfo->infName);
        }
        DeviceEnumInfoList curList = usbDeviceInfoMatch(portInfoList);
        return utils::subtract_sets(curList, deviceInfoList_);
    }
#endif
    return {};
}

DeviceEnumInfoList UsbDeviceEnumerator::usbDeviceInfoMatch(const SourcePortInfoList portInfoList) {
    DeviceEnumInfoList deviceInfoList;
    // todo: match device by pid
    (void)portInfoList;
    return deviceInfoList;
}

void UsbDeviceEnumerator::deviceArrivalHandleThreadFunc() {
    std::mutex                   mtx;
    std::unique_lock<std::mutex> lk(mtx);
    while(!destroy_) {
        newUsbPortArrivalCV_.wait(lk, [&]() { return newUsbPortArrival_ || destroy_; });
        if(destroy_) {
            break;
        }
        uint16_t delayTime = 1000;
#ifdef OS_MACOS
        delayTime = 3000;
#endif
        do {
            newUsbPortArrival_ = false;
            newUsbPortArrivalCV_.wait_for(lk, std::chrono::milliseconds(delayTime));
        } while(!destroy_ && newUsbPortArrival_);

        if(destroy_) {
            break;
        }
        DeviceEnumInfoList addedDevList;
        {
            std::unique_lock<std::recursive_mutex> lock(deviceInfoListMutex_);

            addedDevList = queryArrivalDevice();
            for(auto &item: addedDevList) {
                deviceInfoList_.emplace_back(item);
            }
        }

        if(addedDevList.size()) {
            LOG_DEBUG("device list changed: added={0}, current={1}", addedDevList.size(), deviceInfoList_.size());
            if(!deviceInfoList_.empty()) {
                LOG_DEBUG("Current device list: ");
                for(auto &deviceInfo: deviceInfoList_) {
                    LOG_DEBUG("  - Name: {}, PID: 0x{:04X}, SN/ID: {}", deviceInfo->name_, deviceInfo->pid_, deviceInfo->deviceSn_);
                }
            }
            std::unique_lock<std::mutex> lock(callbackMutex_);
            if(!destroy_ && devChangedCallback_) {
                devChangedCallback_({}, addedDevList);
            }
        }
    }
}

DeviceEnumInfoList UsbDeviceEnumerator::getDeviceInfoList() {
    std::unique_lock<std::recursive_mutex> lock(deviceInfoListMutex_);
    return deviceInfoList_;
}

void UsbDeviceEnumerator::setDeviceChangedCallback(DeviceChangedCallback callback) {
    std::unique_lock<std::mutex> lock(callbackMutex_);
<<<<<<< HEAD
    devChangedCallback_ = [callback, this](const std::vector<std::shared_ptr<DeviceEnumInfo>> &removedList,
                                           const std::vector<std::shared_ptr<DeviceEnumInfo>> &addedList) {
        (void)this;
=======
    devChangedCallback_ = [callback, this](const DeviceEnumInfoList&removedList,
                                           const DeviceEnumInfoList&addedList) {
>>>>>>> 2b5680b (Implementing Context and Device Manager.)
#ifdef __ANDROID__
        // 在安卓平台需要在同线程内回调到java，并在回调函数内完成相关资源释放
        callback(removedList, addedList);
#elif defined(__linux__)
        // 解决linux短时间内多次回调，并且用户回调函数内调用有访问libusb的相关接口导致死锁问题。
        auto cbThread = std::thread(callback, removedList, addedList);
        cbThread.detach();
#else
        // WIN 平台由于回调是MF相关线程调用，如果不切换线程直接回调到用户程序，用户程序需要更新MFC界面，会导致程序崩溃；

        if(devChangedCallbackThread_.joinable()) {
            devChangedCallbackThread_.join();
        }
        // auto cb = [this, removedList, addedList](){
        //     LOG_ERROR("device changed callback begin";
        //     callback(removedList, addedList);
        //     LOG_ERROR("device changed callback end";
        // };
        // devChangedCallbackThread_ = std::thread(cb);
        devChangedCallbackThread_ = std::thread(callback, removedList, addedList);
#endif
    };
}

std::shared_ptr<IDevice> UsbDeviceEnumerator::createDevice(const std::shared_ptr<const DeviceEnumInfo> &info) {
    LOG_DEBUG("UsbDeviceEnumerator createDevice...");
    std::shared_ptr<IDevice> device;

    std::unique_lock<std::recursive_mutex> lock(deviceInfoListMutex_);
<<<<<<< HEAD
    auto                                   info_found =
        std::find_if(deviceInfoList_.begin(), deviceInfoList_.end(), [&](std::shared_ptr<DeviceEnumInfo> item) { return item->uid_ == info->uid_; });
=======
    auto info_found = std::find_if(deviceInfoList_.begin(), deviceInfoList_.end(), [&](const std::shared_ptr<const DeviceEnumInfo>& item) { return item->uid_ == info->uid_; });
>>>>>>> 2b5680b (Implementing Context and Device Manager.)
    if(info_found == deviceInfoList_.end()) {
        return nullptr;
    }

    // todo: create device by pid

    LOG_DEBUG("Device created successfully! Name: {0}, PID: 0x{1:04x}, SN/ID: {2}", info->name_, info->pid_, info->deviceSn_);

    return device;
}

}  // namespace libobsensor
