#pragma once

#include "StringUtils.hpp"
#include <algorithm>
#include <locale>

namespace libobsensor {
namespace utils {

std::vector<std::string> tokenize(const std::string &s, const char separator) {

    std::vector<std::string> ret;
    if(!s.empty()) {
        std::istringstream stream(s);
        std::string        str;
        /**
         * 默认终止符是','
         */
        while(getline(stream, str, separator)) {
            ret.push_back(str);
        }
    }
    return ret;
}

std::vector<std::string> split(const std::string &s, const std::string &separator) {
    std::vector<std::string> res;
    if("" == s)
        return res;
    char *strs = new char[s.length() + 1];
    strcpy(strs, s.c_str());

    char *d = new char[separator.length() + 1];
    strcpy(d, separator.c_str());

    char *p = strtok(strs, d);
    while(p) {
        std::string str = p;
        res.push_back(str);
        p = strtok(NULL, d);
    }
    return res;
}

std::string remove(const std::string &s, const std::string &rm) {
    auto        strSplitVec = split(s, rm);
    std::string outStr      = "";
    for(auto &str: strSplitVec) {
        outStr += str;
    }
    return outStr;
}

std::string replace(const std::string &s, const std::string &src, const std::string &rep) {
    std::string outStr = s;
    auto        place  = outStr.find(src);
    while(place < outStr.length()) {
        outStr = outStr.replace(place, rep.length(), rep);
        place  = outStr.find(src);
    }
    return outStr;
}

char my_tolower(char ch) {
     return static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
}

std::string toLower(const std::string &s) {
    std::string outStr = s;
    std::transform(outStr.begin(), outStr.end(), outStr.begin(), my_tolower);
    return outStr;
}
char my_toupper(char ch) {
     return static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
}

std::string toUpper(const std::string &s) {
    std::string outStr = s;
    std::transform(outStr.begin(), outStr.end(), outStr.begin(), my_toupper);
    return outStr;
}

}  // namespace utils
}  // namespace libobsensor
