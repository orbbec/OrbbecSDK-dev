#pragma once
#include <memory>
#include <string>
#include <mutex>
#include <memory>

namespace libobsensor {
class Context {
private:
    Context(std::string configPath = "");

    static std::mutex             instanceMutex_;
    static std::weak_ptr<Context> instanceWeakPtr_;

public:
    ~Context();
    static std::shared_ptr<Context> getInstance(std::string configPath = "");
    static bool                     isInstanceExist();
};
}  // namespace libobsensor
