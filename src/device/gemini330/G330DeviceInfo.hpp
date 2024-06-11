// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec  Corporation. All Rights Reserved.
#pragma once
#include "core/device/DeviceInfo.hpp"

#include <iostream>
#include <string>
#include <vector>

namespace libobsensor {
namespace g2r {

class G330DeviceInfo : public DeviceInfo {
public:
    G330DeviceInfo(const SourcePortInfoList groupedInfoList);
    ~G330DeviceInfo() noexcept;

    static std::vector<std::shared_ptr<DeviceInfo>> createDeviceInfos(const SourcePortInfoList infoList);
};

}  // namespace g2r
}  // namespace libobsensor
