// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec  Corporation. All Rights Reserved.

#pragma once
#include "IDeviceEnumerator.hpp"

#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <thread>
#include <condition_variable>

namespace libobsensor {

class DeviceManager {
private:
    DeviceManager();

    static std::weak_ptr<DeviceManager> instanceWeakPtr_;
    static std::mutex instanceMutex_;

public:
    static std::shared_ptr<DeviceManager> getInstance();
    ~DeviceManager() noexcept;

    static std::shared_ptr<IDevice> createNetDevice(std::string address, uint16_t port);
    std::shared_ptr<IDevice>        createDevice(const std::shared_ptr<const DeviceEnumInfo> &info);

    DeviceEnumInfoList getDeviceInfoList() const;

    void setDeviceChangedCallback(DeviceChangedCallback callback);

    void enableDeviceClockSync(uint64_t repeatInterval);

    void enableNetDeviceEnumeration(bool enable);
    bool isNetDeviceEnumerationEnable();

private:
    void multiDeviceSyncFunc(uint8_t retry = 0, std::vector<std::string> uids = {});
    void onDeviceChanged(const DeviceEnumInfoList&  removed, const DeviceEnumInfoList&  added);

private:
    bool                        destroy_;

    std::mutex            callbackMutex_;
    DeviceChangedCallback devChangedCallback_ = nullptr;

    std::map<std::string, std::weak_ptr<IDevice>> createdDevices_;
    std::mutex                                    createdDevicesMutex_;

    std::thread             multiDeviceSyncThread_;
    std::condition_variable multiDeviceSyncCv_;
    uint64_t                multiDeviceSyncIntervalMs_;  // unit: ms

    std::vector<std::shared_ptr<IDeviceEnumerator>> deviceEnumerators_;
};
}  // namespace libobsensor

