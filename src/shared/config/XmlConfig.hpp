//
// Created by zyb on 2020/9/16.
//

#pragma once

#include <openobsdk/h/ObTypes.h>

#include <stdarg.h>
#include <algorithm>
#include <memory>
#include <mutex>
#include <vector>
#include <map>


namespace libobsensor {
class XMLDocument;
class XMLElement;
class XMLAttribute;
class XMLComment;
class XMLText;
class XMLDeclaration;
class XMLUnknown;
class XMLPrinter;
class XmlConfig final {

public:
    explicit XmlConfig(std::string configFilePath);
    typedef std::shared_ptr<XmlConfig> Ptr;
    ~XmlConfig();
    XmlConfig(XmlConfig &)                  = delete;
    XmlConfig &operator=(const XmlConfig &) = delete;
    static Ptr getInstance(std::string configFilePath);

public:
    inline bool isLoadConfigFileSuccessful() const {
        return loadFileSuccess_;
    }
    bool loadConfigFile(const std::string &configFilePath);
    bool loadConfigFromBuffer(const char *buffer, size_t size);

public:
    bool isNodeContained(const std::string &nodePathName);
    bool getIntValue(const std::string &nodePathName, int &t);
    bool getBooleanValue(const std::string &nodePathName, bool &t);
    bool getFloatValue(const std::string &nodePathName, float &t);
    bool getDoubleValue(const std::string &nodePathName, double &t);
    bool getStringValue(const std::string &nodePathName, std::string &t);
    XMLElement *getRootElement() const;

    std::string getConfigFilePath() const;

protected:
    bool loadDefaultConfigPath();

private:
    bool getTextOfLeafNode(const std::string &nodePathName, std::string &text);
    bool setTextOfLeafNode(const std::string &nodePathName, const std::string &text);

private:
    static Ptr                  instancePtr_;
    static std::recursive_mutex mutex_;
    static XMLDocument         *doc_;

private:
    bool                                    loadFileSuccess_ = false;
    std::string                             configFilePath_;
    std::vector<std::string>                preDefPaths_;
    XMLElement                             *rootXMLElement_;
};
}  // namespace libobsensor
