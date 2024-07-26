#include "FemtoBoltPropertyPort.hpp"
#include "frameprocessor/FrameProcessor.hpp"
#include "sensor/video/DisparityBasedSensor.hpp"
#include "IDeviceComponent.hpp"

namespace libobsensor {
FemtoBoltPropertyPort::FemtoBoltPropertyPort(IDevice *owner) : owner_(owner) {}

void FemtoBoltPropertyPort::setPropertyValue(uint32_t propertyId, OBPropertyValue value) {
    propertyId     = 0;
    value.intValue = 0;
}

void FemtoBoltPropertyPort::getPropertyValue(uint32_t propertyId, OBPropertyValue *value) {
    propertyId     = 0;
    value->intValue = 0;
}

void FemtoBoltPropertyPort::getPropertyRange(uint32_t propertyId, OBPropertyRange *range) {
    propertyId = 0;
    range->cur  = 0;
}

void FemtoBoltPropertyPort::markOutputDisparityFrame(bool enable) {
    bool = false;
}

}  // namespace libobsensor