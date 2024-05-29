#include "publicFilterLoader.hpp"
#include "FramePixelValueProcess.hpp"

namespace libobsensor {
publicFilterCreator::publicFilterCreator(std::function<std::shared_ptr<IFilter>()> creatorFunc) : creatorFunc_(creatorFunc) {}

std::shared_ptr<IFilter> publicFilterCreator::create() {
    return creatorFunc_();
}


namespace PublicFilterCreatorLoader {
std::map<std::string, std::shared_ptr<IFilterCreator>> getCreators() {
    static std::map<std::string, std::shared_ptr<IFilterCreator>> filterCreators = {
        { "PixelValueScaler#public", std::make_shared<publicFilterCreator>([]() { return std::make_shared<PixelValueScaler>("PixelValueScaler"); }) },
    };
    return filterCreators;
}

}  // namespace publicFilterLoader
}  // namespace libobsensor