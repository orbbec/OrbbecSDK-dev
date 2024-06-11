// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec Corporation. All Rights Reserved.

#pragma once
#include "IProperty.hpp"
#include "ISourcePort.hpp"

namespace libobsensor {

class UvcPropertyPort: public IPropertyPort {
public:
    UvcPropertyPort(const std::shared_ptr<ISourcePort>& backend);
    ~UvcPropertyPort() noexcept override = default;

    void setPropertyValue(uint64_t propertyId, OBPropertyValue value) override;
    void getPropertyValue(uint64_t propertyId, OBPropertyValue *value) override;
    void getPropertyRange(uint64_t propertyId, OBPropertyRange *range) override;

private:
    std::shared_ptr<ISourcePort> backend_;
};

}  // namespace libobsensor
