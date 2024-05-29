#pragma once

#include <sstream>
#include <string>

namespace libobsensor {
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

template <typename T> class isStreamable {
    template <typename S> static auto test(const S *t) -> decltype(std::cout << **t);
    static auto                       test(...) -> std::false_type;

public:
    enum { value = !std::is_same<decltype(test((T *)0)), std::false_type>::value };
};

template <class T, bool S> struct ArgsStreamer {
    void streamArg(std::ostream &out, const T &val, bool last) {
        out << ':' << val << (last ? " " : ", ");
    }
};

template <class T> struct ArgsStreamer<T *, true> {
    void streamArg(std::ostream &out, T *val, bool last) {
        out << ':';  // If pointer not null, stream its content
        if(val)
            out << *val;
        else
            out << "nullptr";
        out << (last ? "" : ", ");
    }
};

template <class T> struct ArgsStreamer<T *, false> {
    void streamArg(std::ostream &out, T *val, bool last) {
        out << ':';  // If pointer is not null, stream the pointer
        if(val)
            out << (int *)val;  // Go through (int*) to avoid dumping the content of char*
        else
            out << "nullptr";
        out << (last ? "" : ", ");
    }
};

template <class T> void ArgsToStream(std::ostream &out, const char *names, const T &last) {
    out << names;
    ArgsStreamer<T, isStreamable<T>::value> s;
    s.streamArg(out, last, true);
}

template <class T, class... U> void ArgsToStream(std::ostream &out, const char *names, const T &first, const U &...rest) {
    while(*names && *names != ',')
        out << *names++;
    ArgsStreamer<T, isStreamable<T>::value> s;
    s.streamArg(out, first, false);
    while(*names && (*names == ',' || isspace(*names)))
        ++names;
    ArgsToStream(out, names, rest...);
}

}  // namespace utils
}  // namespace libobsensor
