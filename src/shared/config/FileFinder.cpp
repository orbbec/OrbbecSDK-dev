//
// Created by zyb on 2020/9/16.
//
#include "FileFinder.hpp"

std::string libobsensor::FileFinder::getCurrentAppPath() {
#if !defined(_WIN32)
    char temp[PATH_MAX];
    if(::getcwd(temp, PATH_MAX) == NULL) {
        return std::string();
    }
    return std::string(temp);
#else
    //    MAX_PATH_WINDOWS
    wchar_t temp[256];
    if(!_wgetcwd(&temp[0], 256)) {
        std::wstring();
    }
    std::wstring wstring(temp);
    return ws2s(wstring);
#endif
}
#if defined(_WIN32)
std::string libobsensor::FileFinder::ws2s(const std::wstring &ws) {
    std::string str;
    if(!ws.empty()) {
        int len = WideCharToMultiByte(CP_ACP, 0, ws.c_str(), (int)ws.size(), NULL, 0, NULL, NULL);
        if(len <= 0)
            return str;

        char *buffer = new char[len + 1];
        if(buffer == NULL)
            return str;

        WideCharToMultiByte(CP_ACP, 0, ws.c_str(), (int)ws.size(), buffer, len, NULL, NULL);
        buffer[len] = '\0';
        str.append(buffer);
        delete[] buffer;
    }
    return str;
}
std::wstring libobsensor::FileFinder::s2ws(const std::string &s) {
    std::wstring ws;
    if(!s.empty()) {
        // 获取缓冲区大小，并申请空间，缓冲区大小按字符计算
        int    len    = MultiByteToWideChar(CP_ACP, 0, s.c_str(), (int)s.size(), NULL, 0);
        TCHAR *buffer = new TCHAR[len + 1];
        // 多字节编码转换成宽字节编码
        MultiByteToWideChar(CP_ACP, 0, s.c_str(), (int)s.size(), buffer, len);
        buffer[len] = '\0';  // 添加字符串结尾
        // 删除缓冲区并返回值
        ws.append(buffer);
        delete[] buffer;
    }
    return ws;
}
#endif
bool libobsensor::FileFinder::isExists(const std::string &Path) {
    if(Path.empty()) {
        return false;
    }
#if defined(_WIN32)
    std::wstring ws = s2ws(Path);
    return GetFileAttributesW(ws.c_str()) != INVALID_FILE_ATTRIBUTES;
#else
    struct stat sb;
    return stat(Path.c_str(), &sb) == 0;
#endif
}

bool libobsensor::FileFinder::isFileExist(const std::string &filePath) {
    if(filePath.empty()) {
        return false;
    }
#if defined(_WIN32)
    std::wstring ws         = s2ws(filePath);
    DWORD        Attributes = GetFileAttributesW(ws.c_str());
    return (Attributes != INVALID_FILE_ATTRIBUTES && (Attributes & FILE_ATTRIBUTE_DIRECTORY) == 0);
#else
    struct stat sb;
    if(stat(filePath.c_str(), &sb))
        return false;
    return S_ISREG(sb.st_mode);
#endif
}
#if defined(_WIN32)
bool libobsensor::FileFinder::isFileExist(const std::wstring &filePath) {
    if(filePath.empty()) {
        return false;
    }
    DWORD Attributes = GetFileAttributesW(filePath.c_str());
    return (Attributes != INVALID_FILE_ATTRIBUTES && (Attributes & FILE_ATTRIBUTE_DIRECTORY) == 0);
}
bool libobsensor::FileFinder::isDirExist(const std::wstring &DirPath) {
    if(DirPath.empty()) {
        return false;
    }
    DWORD Attributes = GetFileAttributesW(DirPath.c_str());
    if(Attributes == INVALID_FILE_ATTRIBUTES)
        return false;
    return (Attributes & FILE_ATTRIBUTE_DIRECTORY) != 0;
}
#endif

bool libobsensor::FileFinder::isDirExist(const std::string &DirPath) {
    if(DirPath.empty()) {
        return false;
    }
#if defined(_WIN32)
    std::wstring ws         = s2ws(DirPath);
    DWORD        Attributes = GetFileAttributesW(ws.c_str());
    if(Attributes == INVALID_FILE_ATTRIBUTES)
        return false;
    return (Attributes & FILE_ATTRIBUTE_DIRECTORY) != 0;
#else
    struct stat sb;
    if(stat(DirPath.c_str(), &sb))
        return false;
    return S_ISDIR(sb.st_mode);
#endif
}
std::string libobsensor::FileFinder::makeRealPath(const std::string &path) {
#if !defined(_WIN32)
    if(!path.empty()) {
        char temp[PATH_MAX];
        if(realpath(path.c_str(), temp) == NULL) {
            return std::string();
        }
        return std::string(temp);
    }
    return std::string();
#else
    if(!path.empty()) {
        std::wstring ws = s2ws(path);
        std::wstring result_temp(256, '\0');
        DWORD        result_length = GetFullPathNameW(ws.c_str(), 256, &result_temp[0], NULL);
        if(result_length == 0) {
            return std::string();
        }
        std::wstring result(result_temp.substr(0, result_length));
        return ws2s(result);
    }
    return std::string();
#endif
}
std::string libobsensor::FileFinder::getFilename(const std::string &filePath, bool isfileExist) {
    bool fileExist = false;
    if(isfileExist) {
        fileExist = isFileExist(filePath);
        if(!fileExist) {
            return std::string();
        }
    }
    if(!filePath.empty()) {
#if !defined(_WIN32)
        std::string::size_type pos = filePath.find_last_of("/");
#else
        std::string::size_type pos = filePath.find_last_of("\\");
#endif
        if(pos == std::string::npos || pos + 1 == filePath.length()) {
            return std::string();
        }
        return filePath.substr(pos + 1);
    }
    return std::string();
}
std::string libobsensor::FileFinder::getFileExtension(const std::string &filePath, bool isfileExist) {
    bool fileExist = false;
    if(isfileExist) {
        fileExist = isFileExist(filePath);
        if(!fileExist) {
            return std::string();
        }
    }
    if(!filePath.empty()) {
        std::string::size_type pos = filePath.find_last_of(".");
        if(pos == std::string::npos || pos + 1 == filePath.length()) {
            return std::string();
        }
        return filePath.substr(pos + 1);
    }
    return std::string();
}
