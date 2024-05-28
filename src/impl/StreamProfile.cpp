#include "ImplTypes.hpp"
#include "openobsdk/h/StreamProfile.h"
#include "openobsdk/h/ObTypes.h"
#include "exception/ObException.hpp"
#include "context/Context.hpp"
#include "stream/StreamProfile.hpp"
#include "stream/StreamProfileFactory.hpp"

#ifdef __cplusplus
extern "C" {
#endif

ob_stream_profile* ob_create_stream_profile(ob_stream_type type, ob_format format,ob_error **error) BEGIN_API_CALL{
    auto profile = libobsensor::StreamProfileFactory::createStreamProfile(type, format);
    auto profileImpl     = new ob_stream_profile();
    profileImpl->profile = profile;
    return profileImpl;
}
HANDLE_EXCEPTIONS_AND_RETURN(nullptr, type, format)

ob_stream_profile* ob_create_video_stream_profile(ob_stream_type type, ob_format format, uint32_t width, uint32_t height, uint32_t fps,ob_error **error) BEGIN_API_CALL{
    auto profile = libobsensor::StreamProfileFactory::createVideoStreamProfile(type, format, width, height, fps);
    auto profileImpl     = new ob_stream_profile();
    profileImpl->profile = profile;
    return profileImpl;
}
HANDLE_EXCEPTIONS_AND_RETURN(nullptr, type, format, width, height, fps)

ob_stream_profile* ob_create_accel_stream_profile(ob_accel_full_scale_range full_scale_range, ob_accel_sample_rate sample_rate,ob_error **error) BEGIN_API_CALL{
    auto profile = libobsensor::StreamProfileFactory::createAccelStreamProfile(full_scale_range, sample_rate);
    auto profileImpl     = new ob_stream_profile();
    profileImpl->profile = profile;
    return profileImpl;
}
HANDLE_EXCEPTIONS_AND_RETURN(nullptr, full_scale_range, sample_rate)

ob_stream_profile* ob_create_gyro_stream_profile(ob_gyro_full_scale_range full_scale_range, ob_gyro_sample_rate sample_rate,ob_error **error) BEGIN_API_CALL{
    auto profile = libobsensor::StreamProfileFactory::createGyroStreamProfile(full_scale_range, sample_rate);
    auto profileImpl     = new ob_stream_profile();
    profileImpl->profile = profile;
    return profileImpl;
}
HANDLE_EXCEPTIONS_AND_RETURN(nullptr, full_scale_range, sample_rate)

ob_format ob_stream_profile_get_format(ob_stream_profile *profile, ob_error **error) BEGIN_API_CALL{
    VALIDATE_NOT_NULL(profile);
    return profile->profile->getFormat();
}
HANDLE_EXCEPTIONS_AND_RETURN(OB_FORMAT_UNKNOWN, profile)

ob_stream_type ob_stream_profile_get_type(ob_stream_profile *profile, ob_error **error) BEGIN_API_CALL{
    VALIDATE_NOT_NULL(profile);
    return profile->profile->getType();
}
HANDLE_EXCEPTIONS_AND_RETURN(OB_STREAM_UNKNOWN, profile)

ob_extrinsic ob_stream_profile_get_extrinsic_to(ob_stream_profile *source, ob_stream_profile *target, ob_error **error) BEGIN_API_CALL{
    VALIDATE_NOT_NULL(source);
    VALIDATE_NOT_NULL(target);
    return source->profile->getExtrinsicTo(target->profile);
}
HANDLE_EXCEPTIONS_AND_RETURN(ob_extrinsic(), source, target)

void ob_stream_profile_set_extrinsic_to(ob_stream_profile *source, ob_stream_profile *target, ob_extrinsic extrinsic, ob_error **error) BEGIN_API_CALL{
    VALIDATE_NOT_NULL(source);
    VALIDATE_NOT_NULL(target);
    auto noneConstProfile = std::const_pointer_cast<libobsensor::StreamProfile>(source->profile);
    noneConstProfile->bindExtrinsicTo(target->profile, extrinsic);
}
HANDLE_EXCEPTIONS_NO_RETURN(source, target /*, extrinsic*/) //TODO: add ob_extrinsic operator<<

uint32_t ob_video_stream_profile_get_fps(ob_stream_profile *profile, ob_error **error) BEGIN_API_CALL{
    VALIDATE_NOT_NULL(profile);
    if(!profile->profile->is<libobsensor::VideoStreamProfile>()) {
        throw libobsensor::unsupported_operation_exception("It's not a video stream profile!");
    }
    auto videoProfile = profile->profile->as<libobsensor::VideoStreamProfile>();
    return videoProfile->getFps();
}
HANDLE_EXCEPTIONS_AND_RETURN(0, profile)

uint32_t ob_video_stream_profile_get_width(ob_stream_profile *profile, ob_error **error) BEGIN_API_CALL{
    VALIDATE_NOT_NULL(profile);
    if(!profile->profile->is<libobsensor::VideoStreamProfile>()) {
        throw libobsensor::unsupported_operation_exception("It's not a video stream profile!");
    }
    auto videoProfile = profile->profile->as<libobsensor::VideoStreamProfile>();
    return videoProfile->getWidth();
}
HANDLE_EXCEPTIONS_AND_RETURN(0, profile)

uint32_t ob_video_stream_profile_get_height(ob_stream_profile *profile, ob_error **error) BEGIN_API_CALL{
    VALIDATE_NOT_NULL(profile);
    if(!profile->profile->is<libobsensor::VideoStreamProfile>()) {
        throw libobsensor::unsupported_operation_exception("It's not a video stream profile!");
    }
    auto videoProfile = profile->profile->as<libobsensor::VideoStreamProfile>();
    return videoProfile->getHeight();
}
HANDLE_EXCEPTIONS_AND_RETURN(0, profile)

ob_camera_intrinsic ob_video_stream_get_intrinsic(ob_stream_profile *profile, ob_error **error) BEGIN_API_CALL{
    VALIDATE_NOT_NULL(profile);
    if(!profile->profile->is<libobsensor::VideoStreamProfile>()) {
        throw libobsensor::unsupported_operation_exception("It's not a video stream profile!");
    }
    auto videoProfile = profile->profile->as<libobsensor::VideoStreamProfile>();
    return videoProfile->getIntrinsic();
}
HANDLE_EXCEPTIONS_AND_RETURN(ob_camera_intrinsic(), profile)

void ob_video_stream_set_intrinsic(ob_stream_profile *profile, ob_camera_intrinsic intrinsic, ob_error **error) BEGIN_API_CALL{
    VALIDATE_NOT_NULL(profile);
    if(!profile->profile->is<libobsensor::VideoStreamProfile>()) {
        throw libobsensor::unsupported_operation_exception("It's not a video stream profile!");
    }
    auto noneConstProfile = std::const_pointer_cast<libobsensor::StreamProfile>(profile->profile);
    auto videoProfile = noneConstProfile->as<libobsensor::VideoStreamProfile>();
    videoProfile->bindIntrinsic(intrinsic);
}
HANDLE_EXCEPTIONS_NO_RETURN(profile/*, intrinsic*/) //TODO: add ob_camera_intrinsic operator<<

ob_camera_distortion ob_video_stream_get_distortion(ob_stream_profile *profile, ob_error **error) BEGIN_API_CALL{
    VALIDATE_NOT_NULL(profile);
    if(!profile->profile->is<libobsensor::VideoStreamProfile>()) {
        throw libobsensor::unsupported_operation_exception("It's not a video stream profile!");
    }
    auto videoProfile = profile->profile->as<libobsensor::VideoStreamProfile>();
    return videoProfile->getDistortion();
}
HANDLE_EXCEPTIONS_AND_RETURN(ob_camera_distortion(), profile)

void ob_video_stream_set_distortion(ob_stream_profile *profile, ob_camera_distortion distortion, ob_error **error) BEGIN_API_CALL{
    VALIDATE_NOT_NULL(profile);
    if(!profile->profile->is<libobsensor::VideoStreamProfile>()) {
        throw libobsensor::unsupported_operation_exception("It's not a video stream profile!");
    }
    auto noneConstProfile = std::const_pointer_cast<libobsensor::StreamProfile>(profile->profile);
    auto videoProfile = noneConstProfile->as<libobsensor::VideoStreamProfile>();
    videoProfile->bindDistortion(distortion);
}
HANDLE_EXCEPTIONS_NO_RETURN(profile/*, distortion*/) //TODO: add ob_camera_distortion operator<<

ob_accel_full_scale_range ob_accel_stream_profile_get_full_scale_range(ob_stream_profile *profile, ob_error **error) BEGIN_API_CALL{
    VALIDATE_NOT_NULL(profile);
    if(!profile->profile->is<libobsensor::AccelStreamProfile>()) {
        throw libobsensor::unsupported_operation_exception("It's not an accel stream profile!");
    }
    auto accelProfile = profile->profile->as<libobsensor::AccelStreamProfile>();
    return accelProfile->getFullScaleRange();
}
HANDLE_EXCEPTIONS_AND_RETURN(ob_accel_full_scale_range(), profile)


ob_accel_sample_rate ob_accel_stream_profile_get_sample_rate(ob_stream_profile *profile, ob_error **error) BEGIN_API_CALL{
    VALIDATE_NOT_NULL(profile);
    if(!profile->profile->is<libobsensor::AccelStreamProfile>()) {
        throw libobsensor::unsupported_operation_exception("It's not an accel stream profile!");
    }
    auto accelProfile = profile->profile->as<libobsensor::AccelStreamProfile>();
    return accelProfile->getSampleRate();
}
HANDLE_EXCEPTIONS_AND_RETURN(ob_accel_sample_rate(), profile)

ob_accel_intrinsic ob_accel_stream_profile_get_intrinsic(ob_stream_profile *profile, ob_error **error) BEGIN_API_CALL{
    VALIDATE_NOT_NULL(profile);
    if(!profile->profile->is<libobsensor::AccelStreamProfile>()) {
        throw libobsensor::unsupported_operation_exception("It's not an accel stream profile!");
    }
    auto accelProfile = profile->profile->as<libobsensor::AccelStreamProfile>();
    return accelProfile->getIntrinsic();
}
HANDLE_EXCEPTIONS_AND_RETURN(ob_accel_intrinsic(), profile)

void ob_accel_stream_profile_set_intrinsic(ob_stream_profile *profile, ob_accel_intrinsic intrinsic, ob_error **error) BEGIN_API_CALL{
    VALIDATE_NOT_NULL(profile);
    if(!profile->profile->is<libobsensor::AccelStreamProfile>()) {
        throw libobsensor::unsupported_operation_exception("It's not an accel stream profile!");
    }
    auto noneConstProfile = std::const_pointer_cast<libobsensor::StreamProfile>(profile->profile);
    auto accelProfile = noneConstProfile->as<libobsensor::AccelStreamProfile>();
    accelProfile->bindIntrinsic(intrinsic);
}
HANDLE_EXCEPTIONS_NO_RETURN(profile/*, intrinsic*/)//TODO: add ob_accel_intrinsic operator<<

ob_gyro_full_scale_range ob_gyro_stream_profile_get_full_scale_range(ob_stream_profile *profile, ob_error **error) BEGIN_API_CALL{
    VALIDATE_NOT_NULL(profile);
    if(!profile->profile->is<libobsensor::GyroStreamProfile>()) {
        throw libobsensor::unsupported_operation_exception("It's not a gyro stream profile!");
    }
    auto gyroProfile = profile->profile->as<libobsensor::GyroStreamProfile>();
    return gyroProfile->getFullScaleRange();
}
HANDLE_EXCEPTIONS_AND_RETURN(ob_gyro_full_scale_range(), profile)

ob_gyro_sample_rate ob_gyro_stream_profile_get_sample_rate(ob_stream_profile *profile, ob_error **error) BEGIN_API_CALL{
    VALIDATE_NOT_NULL(profile);
    if(!profile->profile->is<libobsensor::GyroStreamProfile>()) {
        throw libobsensor::unsupported_operation_exception("It's not a gyro stream profile!");
    }
    auto gyroProfile = profile->profile->as<libobsensor::GyroStreamProfile>();
    return gyroProfile->getSampleRate();
}
HANDLE_EXCEPTIONS_AND_RETURN(ob_gyro_sample_rate(), profile)

ob_gyro_intrinsic ob_gyro_stream_get_intrinsic(ob_stream_profile *profile, ob_error **error) BEGIN_API_CALL{
    VALIDATE_NOT_NULL(profile);
    if(!profile->profile->is<libobsensor::GyroStreamProfile>()) {
        throw libobsensor::unsupported_operation_exception("It's not a gyro stream profile!");
    }
    auto gyroProfile = profile->profile->as<libobsensor::GyroStreamProfile>();
    return gyroProfile->getIntrinsic();
}
HANDLE_EXCEPTIONS_AND_RETURN(ob_gyro_intrinsic(),profile)

void ob_gyro_stream_set_intrinsic(ob_stream_profile *profile, ob_gyro_intrinsic intrinsic, ob_error **error) BEGIN_API_CALL{
    VALIDATE_NOT_NULL(profile);
    if(!profile->profile->is<libobsensor::GyroStreamProfile>()) {
        throw libobsensor::unsupported_operation_exception("It's not a gyro stream profile!");
    }
    auto noneConstProfile = std::const_pointer_cast<libobsensor::StreamProfile>(profile->profile);
    auto gyroProfile = noneConstProfile->as<libobsensor::GyroStreamProfile>();
    gyroProfile->bindIntrinsic(intrinsic);
}
HANDLE_EXCEPTIONS_NO_RETURN(profile/*, intrinsic*/) //TODO: add ob_gyro_intrinsic operator<<

ob_stream_profile *ob_stream_profile_list_get_video_stream_profile(ob_stream_profile_list *profile_list, int width, int height, ob_format format, int fps,ob_error **error) BEGIN_API_CALL{
    VALIDATE_NOT_NULL(profile_list);
    auto matchedProfileList = libobsensor::matchVideoStreamProfile(profile_list->profileList, width, height, fps, format);
    if(matchedProfileList.empty()) {
        throw libobsensor::invalid_value_exception("Invalid input, No matched video stream profile found!");
    }
    auto profileImpl     = new ob_stream_profile();
    profileImpl->profile = matchedProfileList[0];
    return profileImpl;
}
HANDLE_EXCEPTIONS_AND_RETURN(nullptr, profile_list, width, height, format, fps)

ob_stream_profile *ob_stream_profile_list_get_accel_stream_profile(ob_stream_profile_list *profile_list, ob_accel_full_scale_range full_scale_range,ob_accel_sample_rate sample_rate, ob_error **error) BEGIN_API_CALL{
    VALIDATE_NOT_NULL(profile_list);
    auto matchedProfileList = libobsensor::matchAccelStreamProfile(profile_list->profileList, full_scale_range, sample_rate);
    if(matchedProfileList.empty()) {
        throw libobsensor::invalid_value_exception("Invalid input, No matched accel stream profile found!");
    }
    auto profileImpl     = new ob_stream_profile();
    profileImpl->profile = matchedProfileList[0];
    return profileImpl;
}
HANDLE_EXCEPTIONS_AND_RETURN(nullptr, profile_list, full_scale_range, sample_rate)

ob_stream_profile *ob_stream_profile_list_get_gyro_stream_profile(ob_stream_profile_list *profile_list, ob_gyro_full_scale_range full_scale_range,ob_gyro_sample_rate sample_rate, ob_error **error) BEGIN_API_CALL{
    VALIDATE_NOT_NULL(profile_list);
    auto matchedProfileList = libobsensor::matchGyroStreamProfile(profile_list->profileList, full_scale_range, sample_rate);
    if(matchedProfileList.empty()) {
        throw libobsensor::invalid_value_exception("Invalid input, No matched accel stream profile found!");
    }
    auto profileImpl     = new ob_stream_profile();
    profileImpl->profile = matchedProfileList[0];
    return profileImpl;
}
HANDLE_EXCEPTIONS_AND_RETURN(nullptr, profile_list, full_scale_range, sample_rate)

ob_stream_profile *ob_stream_profile_list_get_profile(ob_stream_profile_list *profile_list, int index, ob_error **error) BEGIN_API_CALL{
    VALIDATE_NOT_NULL(profile_list);
    auto innerProfiles = profile_list->profileList;
    if(index < 0 || index >= innerProfiles.size()) {
        throw libobsensor::invalid_value_exception("ob_stream_profile_list_get_profile: index out of range!");
    }
    auto innerProfile = innerProfiles[index];
    auto profileImpl         = new ob_stream_profile();
    profileImpl->profile     = innerProfile;
    return profileImpl;
}
HANDLE_EXCEPTIONS_AND_RETURN(nullptr, profile_list, index)

uint32_t ob_stream_profile_list_count(ob_stream_profile_list *profile_list, ob_error **error) BEGIN_API_CALL{
    VALIDATE_NOT_NULL(profile_list);
    auto innerProfiles = profile_list->profileList;
    return (uint32_t)innerProfiles.size();
}
HANDLE_EXCEPTIONS_AND_RETURN(0, profile_list)

void ob_delete_stream_profile_list(ob_stream_profile_list *profile_list, ob_error **error) BEGIN_API_CALL{
    VALIDATE_NOT_NULL(profile_list);
    delete profile_list;
}
HANDLE_EXCEPTIONS_NO_RETURN(profile_list)

void ob_delete_stream_profile(ob_stream_profile *profile, ob_error **error) BEGIN_API_CALL{
    VALIDATE_NOT_NULL(profile);
    delete profile;
}
HANDLE_EXCEPTIONS_NO_RETURN(profile)

#ifdef __cplusplus
}
#endif