#pragma once

#include <sstream>
#include <string>

namespace ob {
namespace utils {

struct to_string {
    std::ostringstream            ss;
    template <class T> to_string &operator<<(const T &val) {
        ss << val;
        return *this;
    }
    operator std::string() const {
        return ss.str();
    }
};

}  // namespace utils
}  // namespace ob
