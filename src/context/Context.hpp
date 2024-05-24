#pragma once
#include <memory>
#include <string>

namespace libobsensor
{
class Context {
public:
    ~Context();
    static std::shared_ptr<Context> getInstance(std::string configPath = "");
    static bool                     isInstanceExist();

private:
    Context(std::string configPath = "");

private:
};
} // namespace libobsensor
