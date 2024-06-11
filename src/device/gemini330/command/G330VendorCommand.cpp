#include "G330VendorCommand.hpp"
#include <memory>

namespace libobsensor {

G330VendorCommand::G330VendorCommand(std::shared_ptr<HostProtocol> hostProtocol) : VendorCommand(hostProtocol) {}

}  // namespace libobsensor
