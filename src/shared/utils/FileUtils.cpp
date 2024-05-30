#include "FileUtils.hpp"

#include <cstring>
#include <cstdlib>
#include <memory>

#ifdef WIN32
#include <direct.h>
#include <wchar.h>
#include <io.h>
#include <Windows.h>
#include <locale>
#include <codecvt>
#else
#include <sys/stat.h>
#include <unistd.h>
#include <limits.h>
#endif

#include "logger/Logger.hpp"

namespace libobsensor {
namespace utils {
#ifdef WIN32
// static std::string wideCharToUTF8(const WCHAR *s) {
//     auto len = WideCharToMultiByte(CP_UTF8, 0, s, -1, nullptr, 0, nullptr, nullptr);
//     if(len <= 0) {
//         return "";
//     }
//     std::shared_ptr<char> buffer(new char[len + 1]{}, std::default_delete<char[]>());
//     len = WideCharToMultiByte(CP_UTF8, 0, s, -1, buffer.get(), len + 1, nullptr, nullptr);
//     return std::string(buffer.get());
// }

// static std::shared_ptr<WCHAR> utf8ToWideChar(const char *szMultiByteText) {
//     int nChCount = MultiByteToWideChar(CP_UTF8, 0, szMultiByteText, -1, NULL, 0);
//     if(nChCount <= 0) {
//         return nullptr;
//     }
//     std::shared_ptr<WCHAR> wideCharText(new WCHAR[nChCount + 1]{}, std::default_delete<WCHAR[]>());
//     MultiByteToWideChar(CP_UTF8, 0, szMultiByteText, -1, wideCharText.get(), (nChCount + 1) * sizeof(WCHAR));

//     return wideCharText;
// }

// static std::shared_ptr<WCHAR> utf8ToWideChar(const std::string &str) {
//     return utf8ToWideChar(str.c_str());
// }
#endif

bool checkDir(const char *dir) {
#ifdef WIN32
    return (_access(dir, 0) == 0);
#else
    return (access(dir, 0) == 0);
#endif
}

int mkDirs(const char *dir) {
    size_t i, len;
    char   str[512];
    int    error = 0;
#ifdef WIN32
    strncpy_s(str, dir, 512);
    len = strlen(str);
    for(i = 0; i < len; i++) {
        if(str[i] == '/' || str[i] == '\\') {
            str[i] = '\0';
            if(_access(str, 0) != 0) {
                error = _mkdir(str);
                if(error != 0) {
                    return error;
                }
            }
            str[i] = '/';
        }
    }
    if(len > 0 && _access(str, 0) != 0) {
        error = _mkdir(str);
    }
    return error;
#else
    strncpy(str, dir, 512);
    len = strlen(str);
    umask(0000);
    for(i = 0; i < len; i++) {
        if(i > 0 && str[i] == '/') {
            str[i] = '\0';
            if(access(str, 0) != 0) {
                error = mkdir(str, 0755);
                if(error != 0) {
                    return error;
                }
            }
            str[i] = '/';
        }
    }
    if(len > 0 && access(str, 0) != 0) {
        error = mkdir(str, 0755);
    }
    return error;
#endif
}

std::string getCurrentWorkDirectory() {
#ifdef WIN32
    const DWORD size = GetCurrentDirectoryA(0, nullptr);
    std::string pathStr;
    pathStr.resize(size - 1);
    GetCurrentDirectoryA(size, &pathStr[0]);
    return pathStr;
#else
    char cwd[PATH_MAX] = {};
    if(getcwd(cwd, sizeof(cwd)) != NULL) {
        return std::string(cwd);
    }
    else {
        perror("getcwd() error");
        return "";
    }
#endif
}

std::string joinPaths(const std::string &parent, const std::string &fileName) {
    char separator = '/';
#ifdef WIN32
    separator = '\\';
#endif
    std::string rst = parent;
    if(!parent.empty() && parent.back() != separator) {
        rst += separator;
    }

    return rst + fileName;
}

std::string readFile(const std::string &filePath) {
    FILE *pFile = nullptr;
    pFile       = fopen(filePath.c_str(), "rb");
    if(nullptr == pFile) {
        LOG_WARN("open file failed. filePath: {}", filePath);
        return std::string();
    }

    // obtain file size:
    fseek(pFile, 0, SEEK_END);
    long lSize = ftell(pFile);
    rewind(pFile);

    if(lSize <= 0) {
        fclose(pFile);
        return std::string();
    }

    auto bufPtr = std::shared_ptr<char>(new char[lSize + 1], std::default_delete<char[]>());
    memset(bufPtr.get(), 0, lSize + 1);
    auto result = fread(bufPtr.get(), 1, lSize, pFile);

    if(result != lSize) {
        if(0 != feof(pFile)) {
            fclose(pFile);
            return std::string(bufPtr.get());
        }

        int errCode = ferror(pFile);
        if(0 != errCode) {
            LOG_WARN("Read file failed. filePath: {}, errorCode: {}", filePath, errCode);
            fclose(pFile);
            return std::string();
        }
    }

    fclose(pFile);
    return std::string(bufPtr.get());
}

void forEachFileInDirectory(const std::string &directory, const std::function<void(const std::string &)> &callback) {
#ifdef WIN32
    std::wstring_convert<std::codecvt_utf8<wchar_t>> converter;
    std::wstring                                     wdir = converter.from_bytes(directory);
    WIN32_FIND_DATAW                                 findData;
    HANDLE                                           hFind = FindFirstFileW((wdir + L"\\*").c_str(), &findData);
    if(hFind == INVALID_HANDLE_VALUE) {
        return;
    }
    do {
        if(findData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
           continue;
        }
        else {
            std::string fileName = converter.to_bytes(findData.cFileName);
            callback(fileName);
        }
    } while(FindNextFileW(hFind, &findData));
    FindClose(hFind);
#else
    DIR           *dir;
    struct dirent *ent;
    if((dir = opendir(directory.c_str())) != NULL) {
        while((ent = readdir(dir)) != NULL) {
            if(ent->d_type == DT_DIR) {
                if(strcmp(ent->d_name, ".") != 0 && strcmp(ent->d_name, "..") != 0) {
                    std::string subDir = joinPaths(directory, ent->d_name);
                }
            }
            else if(ent->d_type == DT_REG) {
                std::string fileName = joinPaths(directory, ent->d_name);
                callback(fileName);
            }
        }
        closedir(dir);
    }
#endif
}

void forEachSubDirInDirectory(const std::string &directory, const std::function<void(const std::string &)> &callback) {
#ifdef WIN32
    std::wstring_convert<std::codecvt_utf8<wchar_t>> converter;
    std::wstring                                     wdir = converter.from_bytes(directory);
    WIN32_FIND_DATAW                                 findData;
    HANDLE                                           hFind = FindFirstFileW((wdir + L"\\*").c_str(), &findData);
    if(hFind == INVALID_HANDLE_VALUE) {
        return;
    }
    do {
        if(findData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
            if(wcscmp(findData.cFileName, L".") != 0 && wcscmp(findData.cFileName, L"..") != 0) {
                std::string subDir = converter.to_bytes(findData.cFileName);
                callback(subDir);
            }
        }
    } while(FindNextFileW(hFind, &findData));
    FindClose(hFind);
#else
    DIR           *dir;
    struct dirent *ent;
    if((dir = opendir(directory.c_str())) != NULL) {
        while((ent = readdir(dir)) != NULL) {
            if(ent->d_type == DT_DIR) {
                if(strcmp(ent->d_name, ".") != 0 && strcmp(ent->d_name, "..") != 0) {
                    std::string subDir = joinPaths(directory, ent->d_name);
                    callback(subDir);
                }
            }
            closedir(dir);
        }
    }
#endif
}

std::string removeExtensionOfFileName(const std::string &fileName){
    size_t pos = fileName.find_last_of(".");
    if(pos == std::string::npos) {
        return fileName;
    }
    return fileName.substr(0, pos);
}

}  // namespace utils
}  // namespace libobsensor
