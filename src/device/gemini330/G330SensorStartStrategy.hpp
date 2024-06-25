#pragma once
#include "interface/ISensorStartStrategy.hpp"
#include "G330DepthAlgModeManager.hpp"
#include "core/stream/StreamProfile.hpp"
#include "exception/ObException.hpp"
#include "logger/LoggerInterval.hpp"
#include <map>

namespace libobsensor {

class G330SensorStartStrategy : public ISensorStartStrategy {
public:
    G330SensorStartStrategy(std::shared_ptr<G330DepthAlgModeManager> depthAlgModeManager);
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
    std::shared_ptr<G330DepthAlgModeManager>                     depthAlgModeManager_;
};

}  // namespace libobsensor