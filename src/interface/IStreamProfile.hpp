#pragma once
#include <atomic>
#include <memory>
#include <vector>

namespace libobsensor
{
class StreamProfile;

typedef std::vector<std::shared_ptr<const StreamProfile>> StreamProfileList;
typedef std::vector<std::shared_ptr<StreamProfile>>       StreamProfileListUnsafe;

} // namespace libobsensor


#ifdef __cplusplus
extern "C" {
#endif
struct ob_stream_profile_t {
    std::shared_ptr<const libobsensor::StreamProfile> profile;
};

struct ob_stream_profile_list_t {
    std::vector<std::shared_ptr<const libobsensor::StreamProfile>> profileList;
};
#ifdef __cplusplus
}
#endif