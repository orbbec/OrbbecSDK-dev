#ifndef OBSENSOR_FILEFINDER_H
#define OBSENSOR_FILEFINDER_H

#include <cctype>
#include <cstring>
#include <string>

#if defined(__linux)
#include <linux/limits.h>
#endif

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <ShlObj.h>
#include <windows.h>
#else
#if defined(OS_MACOS)
#include <sys/syslimits.h>
#endif
#include <unistd.h>
#endif
#include <sys/stat.h>

namespace libobsensor {
class FileFinder {
public:
    static bool isFileExist(const std::string &filePath);

    static bool isDirExist(const std::string &DirPath);

    static std::string getCurrentAppPath();

    static bool isExists(const std::string &Path);

    static std::string makeRealPath(const std::string &path);

    static std::string getFileExtension(const std::string &filePath, bool isFileExit = false);

    static std::string getFilename(const std::string &filePath, bool isFileExit = false);

public:
#if defined(_WIN32)
    static bool isFileExist(const std::wstring &filePath);
    static bool isDirExist(const std::wstring &DirPath);
#endif

public:
#if defined(_WIN32)
    static std::string  ws2s(const std::wstring &ws);
    static std::wstring s2ws(const std::string &s);
#endif
};
}  // namespace libobsensor

#endif  // OBSENSOR_FILEFINDER_H
