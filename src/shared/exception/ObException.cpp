#include "ObException.hpp"
#include "logger/Logger.hpp"
namespace ob {
recoverable_exception::recoverable_exception(const std::string &msg, ob_exception_type exception_type) noexcept : ob_exception(msg, exception_type) {
    LOG_WARN(msg);
}
}  // namespace ob