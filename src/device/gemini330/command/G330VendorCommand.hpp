#pragma once
#include "core/command/VendorCommand.hpp"

namespace libobsensor {
class G330VendorCommand : public VendorCommand {

public:
    G330VendorCommand(std::shared_ptr<HostProtocol> hostProtocol);
    ~G330VendorCommand() noexcept {};

public:
};
}  // namespace libobsensor