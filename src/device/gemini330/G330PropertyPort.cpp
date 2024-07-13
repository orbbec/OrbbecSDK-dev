#include "G330PropertyPort.hpp"
#include "frameprocessor/FrameProcessor.hpp"
#include "sensor/video/DisparityBasedSensor.hpp"
#include "IDeviceComponent.hpp"

namespace libobsensor {

G330PropertyPort::G330PropertyPort(IDevice *owner) : owner_(owner){}

void G330PropertyPort::setPropertyValue(uint32_t propertyId, OBPropertyValue value) {
    switch (propertyId)
    {
    case OB_PROP_SDK_DISPARITY_TO_DEPTH_BOOL:{
        auto processor = owner_->getComponentT<FrameProcessor>(OB_DEV_COMPONENT_DEPTH_FRAME_PROCESSOR);
        processor->setPropertyValue(propertyId, value);

        // close hw disparity if sw disparity is on
        if(value.intValue == 1){
            OBPropertyValue hwDisparityValue;
            auto commandPort = owner_->getComponentT<IPropertyPort>(OB_DEV_COMPONENT_COMMAND_PORT);
            commandPort->getPropertyValue(OB_PROP_DISPARITY_TO_DEPTH_BOOL, &hwDisparityValue);
            if(hwDisparityValue.intValue == 1){
                hwDisparityValue.intValue = 0;
                commandPort->setPropertyValue(OB_PROP_DISPARITY_TO_DEPTH_BOOL, hwDisparityValue);
            }
        }

        // update convert output frame as disparity frame
        enableConvertOutputFrameAsDisparityFrame(static_cast<bool>(value.intValue));
    }break;
    case OB_PROP_DISPARITY_TO_DEPTH_BOOL:{
        auto commandPort = owner_->getComponentT<IPropertyPort>(OB_DEV_COMPONENT_COMMAND_PORT);
        commandPort->setPropertyValue(propertyId, value);

        // update sw disparity status
        OBPropertyValue swDisparityValue;
        auto processor = owner_->getComponentT<FrameProcessor>(OB_DEV_COMPONENT_DEPTH_FRAME_PROCESSOR);
        processor->getPropertyValue(propertyId, &swDisparityValue);
        swDisparityValue.intValue = value.intValue == 1 ? 0 : 1;
        processor->setPropertyValue(OB_PROP_SDK_DISPARITY_TO_DEPTH_BOOL, swDisparityValue);

        // update convert output frame as disparity frame
        enableConvertOutputFrameAsDisparityFrame(static_cast<bool>(swDisparityValue.intValue));
    }break;
    case OB_PROP_DEPTH_UNIT_FLEXIBLE_ADJUSTMENT_FLOAT:{
        auto processor = owner_->getComponentT<FrameProcessor>(OB_DEV_COMPONENT_DEPTH_FRAME_PROCESSOR);
        OBPropertyValue swDisparityEnable;
        processor->getPropertyValue(OB_PROP_SDK_DISPARITY_TO_DEPTH_BOOL, &swDisparityEnable);
        if(swDisparityEnable.intValue == 1){
            processor->setPropertyValue(propertyId, value);
        }else{
            auto commandPort = owner_->getComponentT<IPropertyPort>(OB_DEV_COMPONENT_COMMAND_PORT);
            commandPort->setPropertyValue(propertyId, value);
        }

        // update depth unit
        auto sensor = owner_->getComponentT<ISensor>(OB_DEV_COMPONENT_DEPTH_SENSOR).get();
        auto disparitySensor = std::dynamic_pointer_cast<DisparityBasedSensor>(sensor);
        if(disparitySensor){
            disparitySensor->setDepthUnit(value.floatValue);
        }

    }break;
    
    default:{
        auto commandPort = owner_->getComponentT<IPropertyPort>(OB_DEV_COMPONENT_COMMAND_PORT);
        commandPort->setPropertyValue(propertyId, value);
    }break;

    }
}

void G330PropertyPort::getPropertyValue(uint32_t propertyId, OBPropertyValue *value){
    switch(propertyId){
    case OB_PROP_SDK_DISPARITY_TO_DEPTH_BOOL:{
        auto processor = owner_->getComponentT<FrameProcessor>(OB_DEV_COMPONENT_DEPTH_FRAME_PROCESSOR);
        processor->getPropertyValue(propertyId, value);
    }break;
    case OB_PROP_DEPTH_UNIT_FLEXIBLE_ADJUSTMENT_FLOAT:{
        auto processor = owner_->getComponentT<FrameProcessor>(OB_DEV_COMPONENT_DEPTH_FRAME_PROCESSOR);
        OBPropertyValue swDisparityEnable;
        processor->getPropertyValue(OB_PROP_SDK_DISPARITY_TO_DEPTH_BOOL, &swDisparityEnable);
        if(swDisparityEnable.intValue == 1){            
            processor->getPropertyValue(propertyId, value);
        }else{
            auto commandPort = owner_->getComponentT<IPropertyPort>(OB_DEV_COMPONENT_COMMAND_PORT);
            commandPort->getPropertyValue(propertyId, value);
        }
    }break;
    default:{
        auto commandPort = owner_->getComponentT<IPropertyPort>(OB_DEV_COMPONENT_COMMAND_PORT);
        commandPort->getPropertyValue(propertyId, value);
    }break;

    }
}

void G330PropertyPort::getPropertyRange(uint32_t propertyId, OBPropertyRange *range){
    switch(propertyId){
    case OB_PROP_SDK_DISPARITY_TO_DEPTH_BOOL:{
        auto processor = owner_->getComponentT<FrameProcessor>(OB_DEV_COMPONENT_DEPTH_FRAME_PROCESSOR);
        processor->getPropertyRange(propertyId, range);
    }break;
    case OB_PROP_DEPTH_UNIT_FLEXIBLE_ADJUSTMENT_FLOAT:{
        auto processor = owner_->getComponentT<FrameProcessor>(OB_DEV_COMPONENT_DEPTH_FRAME_PROCESSOR);
        OBPropertyValue swDisparityEnable;
        processor->getPropertyValue(OB_PROP_SDK_DISPARITY_TO_DEPTH_BOOL, &swDisparityEnable);
        if(swDisparityEnable.intValue == 1){            
            processor->getPropertyRange(propertyId, range);
        }else{
            auto commandPort = owner_->getComponentT<IPropertyPort>(OB_DEV_COMPONENT_COMMAND_PORT);
            commandPort->getPropertyRange(propertyId, range);
        }
    }break;
    default:{
            auto commandPort = owner_->getComponentT<IPropertyPort>(OB_DEV_COMPONENT_COMMAND_PORT);
            commandPort->getPropertyRange(propertyId, range);
    }break;
    }
}

void G330PropertyPort::enableConvertOutputFrameAsDisparityFrame(bool enable){
    auto sensor = owner_->getComponentT<ISensor>(OB_DEV_COMPONENT_DEPTH_SENSOR).get();
    auto disparitySensor = std::dynamic_pointer_cast<DisparityBasedSensor>(sensor);
    if(disparitySensor){
        disparitySensor->enableConvertOutputFrameAsDisparityFrame(enable);
    }
}

}