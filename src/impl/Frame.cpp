#include "ImplTypes.hpp"
#include "openobsdk/h/Frame.h"
#include "exception/ObException.hpp"
#include "frame/FrameFactory.hpp"
#include "context/Context.hpp"

#ifdef __cplusplus
extern "C" {
#endif

uint64_t ob_frame_get_index(ob_frame *frame, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(frame);
    return frame->frame->getNumber();
}
HANDLE_EXCEPTIONS_AND_RETURN(uint64_t(0), frame)

uint32_t ob_video_frame_get_width(ob_frame *frame, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(frame);
    if(!frame->frame->is<libobsensor::VideoFrame>()) {
        throw libobsensor::unsupported_operation_exception("It's not a video frame!");
    }
    return frame->frame->as<libobsensor::VideoFrame>()->getWidth();
}
HANDLE_EXCEPTIONS_AND_RETURN(uint32_t(0), frame)

uint32_t ob_video_frame_get_height(ob_frame *frame, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(frame);
    if(!frame->frame->is<libobsensor::VideoFrame>()) {
        throw libobsensor::unsupported_operation_exception("It's not a video frame!");
    }
    return frame->frame->as<libobsensor::VideoFrame>()->getHeight();
}
HANDLE_EXCEPTIONS_AND_RETURN(uint32_t(0), frame)

ob_format ob_frame_get_format(ob_frame *frame, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(frame);
    return frame->frame->getFormat();
}
HANDLE_EXCEPTIONS_AND_RETURN(OB_FORMAT_UNKNOWN, frame)

ob_frame_type ob_frame_get_type(ob_frame *frame, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(frame);
    return frame->frame->getType();
}
HANDLE_EXCEPTIONS_AND_RETURN(OB_FRAME_VIDEO, frame)

uint64_t ob_frame_get_timestamp_ms(ob_frame *frame, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(frame);
    return frame->frame->getTimeStampUsec() / 1000;
}
HANDLE_EXCEPTIONS_AND_RETURN(uint64_t(0), frame)

uint64_t ob_frame_get_timestamp_us(ob_frame *frame, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(frame);
    return frame->frame->getTimeStampUsec();
}
HANDLE_EXCEPTIONS_AND_RETURN(uint64_t(0), frame)

uint64_t ob_frame_get_system_timestamp_ms(ob_frame *frame, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(frame);
    return frame->frame->getSystemTimeStampUsec() / 1000;
}
HANDLE_EXCEPTIONS_AND_RETURN(uint64_t(0), frame)

uint64_t ob_frame_get_system_timestamp_us(ob_frame *frame, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(frame);
    return frame->frame->getSystemTimeStampUsec();
}
HANDLE_EXCEPTIONS_AND_RETURN(uint64_t(0), frame)

uint64_t ob_frame_get_global_timestamp_us(ob_frame *frame, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(frame);
    return frame->frame->getGlobalTimeStampUsec();
}
HANDLE_EXCEPTIONS_AND_RETURN(uint64_t(0), frame)

void ob_frame_set_system_timestamp_us(ob_frame *frame, uint64_t timestamp_us, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(frame);
    auto innerFrame = frame->frame;
    innerFrame->setSystemTimeStampUsec(timestamp_us);
}
HANDLE_EXCEPTIONS_NO_RETURN(frame)

void ob_frame_set_timestamp_us(ob_frame *frame, uint64_t timestamp_us, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(frame);
    auto innerFrame = frame->frame;
    innerFrame->setTimeStampUsec(timestamp_us);
}
HANDLE_EXCEPTIONS_NO_RETURN(frame)

float ob_depth_frame_get_value_scale(ob_frame *frame, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(frame);
    auto depthFrame = frame->frame->as<libobsensor::DepthFrame>();
    return depthFrame->getValueScale();
}
HANDLE_EXCEPTIONS_AND_RETURN(-1.0f, frame)

float ob_points_frame_get_coordinate_value_scale(ob_frame *frame, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(frame);
    auto pointsFrame = frame->frame->as<libobsensor::PointsFrame>();
    return pointsFrame->getCoordinateValueScale();
}
HANDLE_EXCEPTIONS_AND_RETURN(-1.0f, frame)

uint8_t *ob_frame_get_data(ob_frame *frame, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(frame);
    return const_cast<uint8_t *>(frame->frame->getData());
}
HANDLE_EXCEPTIONS_AND_RETURN(nullptr, frame)

uint32_t ob_frame_get_data_size(ob_frame *frame, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(frame);
    return (uint32_t)frame->frame->getDataSize();
}
HANDLE_EXCEPTIONS_AND_RETURN(uint32_t(0), frame)

uint8_t *ob_frame_get_metadata(ob_frame *frame, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(frame);
    return const_cast<uint8_t *>(frame->frame->getMetadata());
}
HANDLE_EXCEPTIONS_AND_RETURN(nullptr, frame)

uint32_t ob_frame_get_metadata_size(ob_frame *frame, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(frame);
    return (uint32_t)frame->frame->getMetadataSize();
}
HANDLE_EXCEPTIONS_AND_RETURN(uint32_t(0), frame)

bool ob_frame_has_metadata(ob_frame *frame, ob_frame_metadata_type type, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(frame);
    return frame->frame->hasMetadata(type);
}
HANDLE_EXCEPTIONS_AND_RETURN(false, frame)

int64_t ob_frame_get_metadata_value(ob_frame *frame, ob_frame_metadata_type type, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(frame);
    return frame->frame->as<libobsensor::VideoFrame>()->getMetadataValue(type);
}
HANDLE_EXCEPTIONS_AND_RETURN(-1, frame)

ob_stream_profile *ob_frame_get_stream_profile(ob_frame *frame, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(frame);
    auto innerProfile = frame->frame->getStreamProfile();
    if(!innerProfile) {
        return NULL;
    }
    auto impl     = new ob_stream_profile();
    impl->ctx     = frame->ctx;
    impl->profile = innerProfile;
    return impl;
}
HANDLE_EXCEPTIONS_AND_RETURN(nullptr, frame)

void ob_frame_set_stream_profile(ob_frame *frame, ob_stream_profile *stream_profile, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(frame);
    VALIDATE_NOT_NULL(stream_profile);
    auto innerFrame   = frame->frame;
    auto innerProfile = stream_profile->profile;
    innerFrame->setStreamProfile(innerProfile);
}
HANDLE_EXCEPTIONS_NO_RETURN(frame, stream_profile)

ob_sensor *ob_frame_get_sensor(ob_frame *frame, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(frame);
    auto innerProfile = frame->frame->getStreamProfile();
    if(!innerProfile) {
        return NULL;
    }

    auto innerSensor = innerProfile->getOwner();
    if(!innerSensor) {
        return NULL;
    }

    auto innerDevice = innerSensor->getOwner();
    if(!innerDevice) {
        return NULL;
    }

    auto sensorImpl    = new ob_sensor();
    sensorImpl->ctx    = frame->ctx;
    sensorImpl->device = innerDevice;
    sensorImpl->type   = innerSensor->getSensorType();
    return sensorImpl;
}
HANDLE_EXCEPTIONS_AND_RETURN(nullptr, frame)

ob_device *ob_frame_get_device(ob_frame *frame, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(frame);
    auto innerProfile = frame->frame->getStreamProfile();
    if(!innerProfile) {
        return NULL;
    }

    auto innerSensor = innerProfile->getOwner();
    if(!innerSensor) {
        return NULL;
    }

    auto innerDevice = innerSensor->getOwner();
    if(!innerDevice) {
        return NULL;
    }

    auto deviceImpl    = new ob_device();
    deviceImpl->ctx    = frame->ctx;
    deviceImpl->device = innerDevice;
    return deviceImpl;
}
HANDLE_EXCEPTIONS_AND_RETURN(nullptr, frame)

uint8_t ob_video_frame_get_pixel_available_bit_size(ob_frame *frame, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(frame);
    return frame->frame->as<libobsensor::VideoFrame>()->getPixelAvailableBitSize();
}
HANDLE_EXCEPTIONS_AND_RETURN(uint8_t(0), frame)

ob_sensor_type ob_ir_frame_get_data_source(ob_frame *frame, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(frame);
    auto type = frame->frame->as<libobsensor::VideoFrame>()->getType();
    if(type == OB_FRAME_IR) {
        return OB_SENSOR_IR;
    }
    if(type == OB_FRAME_IR_LEFT) {
        return OB_SENSOR_IR_LEFT;
    }
    if(type == OB_FRAME_IR_RIGHT) {
        return OB_SENSOR_IR_RIGHT;
    }
    throw libobsensor::invalid_value_exception("invalid ir frame type type: " + std::to_string(type));
}
HANDLE_EXCEPTIONS_AND_RETURN(OB_SENSOR_UNKNOWN, frame)

ob_accel_value ob_accel_frame_get_value(ob_frame *frame, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(frame);
    if(!frame->frame->is<libobsensor::AccelFrame>()) {
        throw libobsensor::unsupported_operation_exception("It's not a accel frame!");
    }
    return frame->frame->as<libobsensor::AccelFrame>()->value();
}
HANDLE_EXCEPTIONS_AND_RETURN(ob_accel_value(), frame)

float ob_accel_frame_get_temperature(ob_frame *frame, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(frame);
    if(!frame->frame->is<libobsensor::AccelFrame>()) {
        throw libobsensor::unsupported_operation_exception("It's not a accel frame!");
    }
    return frame->frame->as<libobsensor::AccelFrame>()->temperature();
}
HANDLE_EXCEPTIONS_AND_RETURN(0.0f, frame)

ob_gyro_value ob_gyro_frame_get_value(ob_frame *frame, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(frame);
    if(!frame->frame->is<libobsensor::GyroFrame>()) {
        throw libobsensor::unsupported_operation_exception("It's not a gyro frame!");
    }
    return frame->frame->as<libobsensor::GyroFrame>()->value();
}
HANDLE_EXCEPTIONS_AND_RETURN(ob_gyro_value(), frame)

float ob_gyro_frame_get_temperature(ob_frame *frame, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(frame);
    if(!frame->frame->is<libobsensor::GyroFrame>()) {
        throw libobsensor::unsupported_operation_exception("It's not a gyro frame!");
    }
    return frame->frame->as<libobsensor::GyroFrame>()->temperature();
}
HANDLE_EXCEPTIONS_AND_RETURN(0.0f, frame)

uint32_t ob_frameset_get_frame_count(ob_frame *frameset, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(frameset);
    if(!frameset->frame->is<libobsensor::FrameSet>()) {
        throw libobsensor::unsupported_operation_exception("It's not a frameset!");
    }
    return frameset->frame->as<libobsensor::FrameSet>()->getFrameCount();
}
HANDLE_EXCEPTIONS_AND_RETURN(uint32_t(0), frameset)

ob_frame *ob_frameset_get_depth_frame(ob_frame *frameset, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(frameset);
    if(!frameset->frame->is<libobsensor::FrameSet>()) {
        throw libobsensor::unsupported_operation_exception("It's not a frameset!");
    }
    auto innerFrame = frameset->frame->as<libobsensor::FrameSet>()->getDepthFrame();
    if(innerFrame == nullptr) {
        return nullptr;
    }
    auto impl   = new ob_frame();
    impl->ctx   = frameset->ctx;
    impl->frame = innerFrame;
    return impl;
}
HANDLE_EXCEPTIONS_AND_RETURN(nullptr, frameset)

ob_frame *ob_frameset_get_color_frame(ob_frame *frameset, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(frameset);
    if(!frameset->frame->is<libobsensor::FrameSet>()) {
        throw libobsensor::unsupported_operation_exception("It's not a frameset!");
    }
    auto innerFrame = frameset->frame->as<libobsensor::FrameSet>()->getColorFrame();
    if(innerFrame == nullptr) {
        return nullptr;
    }
    auto impl   = new ob_frame();
    impl->ctx   = frameset->ctx;
    impl->frame = innerFrame;
    return impl;
}
HANDLE_EXCEPTIONS_AND_RETURN(nullptr, frameset)

ob_frame *ob_frameset_get_ir_frame(ob_frame *frameset, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(frameset);
    if(!frameset->frame->is<libobsensor::FrameSet>()) {
        throw libobsensor::unsupported_operation_exception("It's not a frameset!");
    }
    auto innerFrame = frameset->frame->as<libobsensor::FrameSet>()->getIRFrame();
    if(innerFrame == nullptr) {
        return nullptr;
    }
    auto impl   = new ob_frame();
    impl->ctx   = frameset->ctx;
    impl->frame = innerFrame;
    return impl;
}
HANDLE_EXCEPTIONS_AND_RETURN(nullptr, frameset)

ob_frame *ob_frameset_get_points_frame(ob_frame *frameset, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(frameset);
    if(!frameset->frame->is<libobsensor::FrameSet>()) {
        throw libobsensor::unsupported_operation_exception("It's not a frameset!");
    }
    auto innerFrame = frameset->frame->as<libobsensor::FrameSet>()->getPointsFrame();
    if(innerFrame == nullptr) {
        return nullptr;
    }
    auto impl   = new ob_frame();
    impl->ctx   = frameset->ctx;
    impl->frame = innerFrame;
    return impl;
}
HANDLE_EXCEPTIONS_AND_RETURN(nullptr, frameset)

ob_frame *ob_frameset_get_frame(ob_frame *frameset, ob_frame_type frame_type, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(frameset);
    if(!frameset->frame->is<libobsensor::FrameSet>()) {
        throw libobsensor::unsupported_operation_exception("It's not a frameset!");
    }
    auto innerFrame = frameset->frame->as<libobsensor::FrameSet>()->getFrame(frame_type);
    if(innerFrame == nullptr) {
        return nullptr;
    }
    auto impl   = new ob_frame();
    impl->ctx   = frameset->ctx;
    impl->frame = innerFrame;
    return impl;
}
HANDLE_EXCEPTIONS_AND_RETURN(nullptr, frameset)

ob_frame *ob_frameset_get_frame_by_index(ob_frame *frameset, int index, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(frameset);
    if(!frameset->frame->is<libobsensor::FrameSet>()) {
        throw libobsensor::unsupported_operation_exception("It's not a frameset!");
    }
    auto innerFrame = frameset->frame->as<libobsensor::FrameSet>()->getFrame(index);
    if(innerFrame == nullptr) {
        return nullptr;
    }
    auto impl   = new ob_frame();
    impl->ctx   = frameset->ctx;
    impl->frame = innerFrame;
    return impl;
}
HANDLE_EXCEPTIONS_AND_RETURN(nullptr, frameset)

void ob_frameset_push_frame(ob_frame *frameset, ob_frame *frame, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(frameset);
    VALIDATE_NOT_NULL(frame);
    auto innerFrameSet = frameset->frame->as<libobsensor::FrameSet>();
    auto innerFrame    = frame->frame;
    innerFrameSet->pushFrame(std::move(innerFrame));  // TODO: check frame type
}
HANDLE_EXCEPTIONS_NO_RETURN(frameset, frame)

void ob_delete_frame(ob_frame *frame, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(frame);
    auto innerFrame = frame->frame;
    if(frame->refCnt > 1) {
        frame->refCnt -= 1;
        return;
    }
    delete frame;
}
HANDLE_EXCEPTIONS_NO_RETURN(frame)

void ob_frame_add_ref(ob_frame *frame, ob_error **error) BEGIN_API_CALL {
    VALIDATE_NOT_NULL(frame);
    frame->refCnt += 1;
}
HANDLE_EXCEPTIONS_NO_RETURN(frame)

ob_frame *ob_create_video_frame(ob_frame_type frame_type, ob_format format, uint32_t width, uint32_t height, uint32_t stride_bytes,
                                ob_error **error) BEGIN_API_CALL {

    auto innerFrame = libobsensor::FrameFactory::createVideoFrame(frame_type, format, width, height, stride_bytes);
    if(innerFrame == nullptr) {
        LOG_ERROR("User custom frame create failed!");
        return nullptr;
    }

    auto frameImpl   = new ob_frame();
    frameImpl->ctx   = libobsensor::Context::getInstance();
    frameImpl->frame = innerFrame;

    return frameImpl;
}
NO_ARGS_HANDLE_EXCEPTIONS_AND_RETURN(nullptr)

ob_frame *ob_create_video_frame_from_buffer(ob_frame_type frame_type, ob_format format, uint32_t width, uint32_t height, uint32_t stride_bytes, uint8_t *buffer,
                                            uint32_t buffer_size, ob_frame_destroy_callback *buffer_destroy_cb, void *buffer_destroy_context,
                                            ob_error **error) BEGIN_API_CALL {
    auto innerFrame = libobsensor::FrameFactory::createVideoFrameFromUserBuffer(
        frame_type, format, width, height, stride_bytes, buffer, buffer_size,
        [buffer_destroy_cb, buffer, buffer_destroy_context]() { buffer_destroy_cb(buffer, buffer_destroy_context); });
    if(innerFrame == nullptr) {
        return nullptr;
    }

    auto frameImpl   = new ob_frame();
    frameImpl->ctx   = libobsensor::Context::getInstance();
    frameImpl->frame = innerFrame;

    return frameImpl;
}
NO_ARGS_HANDLE_EXCEPTIONS_AND_RETURN(nullptr)

ob_frame *ob_create_frameset(ob_error **error) BEGIN_API_CALL {
    auto innerFrameSet = libobsensor::FrameFactory::createFrameSet();
    if(innerFrameSet == nullptr) {
        return nullptr;
    }

    auto frameImpl   = new ob_frame();
    frameImpl->ctx   = libobsensor::Context::getInstance();
    frameImpl->frame = innerFrameSet;

    return frameImpl;
}
NO_ARGS_HANDLE_EXCEPTIONS_AND_RETURN(nullptr)

#ifdef __cplusplus
}
#endif