#pragma once
#include <string>
#include <functional>

namespace libobsensor {
namespace utils {
bool        checkDir(const char *dir);
int         mkDirs(const char *dir);
std::string getCurrentWorkDirectory();
std::string joinPaths(const std::string &parent, const std::string &fileName);
std::string readFile(const std::string &filePath);
void forEachFileInDirectory(const std::string &directory, const std::function<void(const std::string &)> &callback);
void forEachSubDirInDirectory(const std::string &directory, const std::function<void(const std::string &)> &callback);
std::string removeExtensionOfFileName(const std::string &fileName);

std::string makeRealPath(const std::string &path);

}  // namespace utils
}  // namespace libobsensor
