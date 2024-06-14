// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec  Corporation. All Rights Reserved.
#pragma once
#include "IDeviceEnumerator.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <memory>

namespace libobsensor {


class G330DeviceInfo : public DeviceEnumInfo, public std::enable_shared_from_this<G330DeviceInfo>{
public:
    G330DeviceInfo( const SourcePortInfoList groupedInfoList);
    ~G330DeviceInfo() noexcept;

    std::shared_ptr<IDevice> createDevice() const override;


    static std::vector<std::shared_ptr<DeviceEnumInfo>> createDeviceInfos( const SourcePortInfoList infoList);
};


}  // namespace libobsensor
