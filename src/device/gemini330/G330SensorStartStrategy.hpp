#pragma once

#include "interface/ISensorStartStrategy.hpp"
#include "core/device/IDevice.hpp"

namespace libobsensor {
namespace g2r {

class G330SensorStartStrategy : public ISensorStartStrategy {
public:
    G330SensorStartStrategy(std::weak_ptr<IDevice> device);
    virtual ~G330SensorStartStrategy() noexcept;

private:
    void initializeValidateSensor();

public:
    bool validateSensorStart(OBSensorType sensorType, std::shared_ptr<const StreamProfile> sp);
    void clearValidateSensor(OBSensorType sensorType);
    bool validatePresetProfile(OBSensorType sensorType, std::shared_ptr<const StreamProfile> streamProfile);

private:
    std::recursive_mutex                                         validate_mutex_;
    std::vector<OBSensorType>                                    sensorTypeList_;
    std::map<OBSensorType, std::shared_ptr<const StreamProfile>> streamProfileMap_;
    std::weak_ptr<IDevice>                                       device_;
};

}  // namespace g2r
}  // namespace libobsensor