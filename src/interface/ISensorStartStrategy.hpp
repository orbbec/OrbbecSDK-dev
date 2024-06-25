#pragma once
#include "libobsensor/h/ObTypes.h"
#include "IStreamProfile.hpp"
#include <memory>

namespace libobsensor {

class ISensorStartStrategy {
public:
    virtual ~ISensorStartStrategy()                                                                      = default;
    virtual bool validatePresetProfile(OBSensorType sensorType, std::shared_ptr<const StreamProfile> sp) = 0;
    virtual bool validateSensorStart(OBSensorType sensorType, std::shared_ptr<const StreamProfile> sp)   = 0;
    virtual void clearValidateSensor(OBSensorType sensorType)                                            = 0;
};

}  // namespace libobsensor