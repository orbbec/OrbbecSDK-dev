#pragma once

#include "openobsdk/h/ObTypes.h"
#include "StreamProfile.hpp"
#include <map>
#include <memory>
#include <vector>

namespace ob {
namespace core {

class StreamExtrinsicsManager {
public:
    static std::shared_ptr<StreamExtrinsicsManager> getInstance();
    static void destroyInstance();

    ~StreamExtrinsicsManager() noexcept;

    void registerExtrinsics(std::shared_ptr<const StreamProfile> from, std::shared_ptr<const StreamProfile> to, const OBExtrinsic &extrinsics);
    void registerSameExtrinsics(std::shared_ptr<const StreamProfile> from, std::shared_ptr<const StreamProfile> to);

    bool        hasExtrinsics(std::shared_ptr<const StreamProfile> from, std::shared_ptr<const StreamProfile> to) const;
    OBExtrinsic getExtrinsics(std::shared_ptr<const StreamProfile> from, std::shared_ptr<const StreamProfile> to);

private:
    StreamExtrinsicsManager();
    void cleanExpiredStreamProfiles();
    void eraseStreamProfile(std::shared_ptr<const StreamProfile> sp);
    void eraseNodeFromExtrinsicsGraph(uint64_t id);

    bool searchPath(std::vector<std::pair<uint64_t, OBExtrinsic>> &path, uint64_t fromId, uint64_t toId) const;

    uint64_t getOrRegisterStreamProfileId(std::shared_ptr<const StreamProfile> profile);
    uint64_t getStreamProfileId(std::shared_ptr<const StreamProfile> profile) const;

private:
    std::map<uint64_t, std::vector<std::weak_ptr<const StreamProfile>>> streamProfileMap_;  // vertices
    std::map<uint64_t, std::vector<std::pair<uint64_t, OBExtrinsic>>>   extrinsicsGraph_;   // graph adjacency list
    uint64_t                                                            nextId_;

    std::recursive_mutex mutex_;
};

}  // namespace core
}  // namespace ob