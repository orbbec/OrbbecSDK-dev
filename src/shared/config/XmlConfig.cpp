#include "XmlConfig.hpp"
#include "tinyxml2.hpp"
#include "FileFinder.hpp"
#include "utils/StringUtils.hpp"
#include "logger/Logger.hpp"

namespace libobsensor {
#if defined(_WIN32)
std::string Slash     = "\\";
std::string LastSlash = "..\\";
#else
std::string Slash     = "/";
std::string LastSlash = "../";
#endif

XmlConfig::Ptr            XmlConfig::instancePtr_ = nullptr;
std::recursive_mutex      XmlConfig::mutex_;
libobsensor::XMLDocument *XmlConfig::doc_         = nullptr;
const char               *CameraConfigurationName = "OrbbecSDKConfig_v1.0.xml";

XmlConfig::XmlConfig(std::string configFilePath) {
    doc_         = new libobsensor::XMLDocument();
    preDefPaths_.clear();
    rootXMLElement_ = nullptr;
    if(configFilePath == "") {
        loadFileSuccess_ = loadDefaultConfigPath();
    }
    else {
        loadFileSuccess_ = loadConfigFile(configFilePath);
    }
}
XmlConfig::~XmlConfig() {
    if(doc_) {
        delete doc_;
    }
}
bool XmlConfig::loadDefaultConfigPath() {
    bool loadSuccessfully = false;
    configFilePath_       = FileFinder::getCurrentAppPath();
    preDefPaths_.push_back(FileFinder::makeRealPath(configFilePath_ + Slash + std::string(CameraConfigurationName)));
    preDefPaths_.push_back(FileFinder::makeRealPath(configFilePath_ + Slash + "Data" + Slash + std::string(CameraConfigurationName)));
    preDefPaths_.push_back(FileFinder::makeRealPath(configFilePath_ + Slash + "data" + Slash + std::string(CameraConfigurationName)));
    preDefPaths_.push_back(FileFinder::makeRealPath(configFilePath_ + Slash + "config" + Slash + std::string(CameraConfigurationName)));
    preDefPaths_.push_back(FileFinder::makeRealPath(configFilePath_ + Slash + "Config" + Slash + std::string(CameraConfigurationName)));
    preDefPaths_.push_back(FileFinder::makeRealPath(configFilePath_ + Slash + LastSlash + std::string(CameraConfigurationName)));
    preDefPaths_.push_back(FileFinder::makeRealPath(configFilePath_ + Slash + LastSlash + "Data" + Slash + std::string(CameraConfigurationName)));
    preDefPaths_.push_back(FileFinder::makeRealPath(configFilePath_ + Slash + LastSlash + "data" + Slash + std::string(CameraConfigurationName)));
    preDefPaths_.push_back(FileFinder::makeRealPath(configFilePath_ + Slash + LastSlash + "config" + Slash + std::string(CameraConfigurationName)));
    preDefPaths_.push_back(FileFinder::makeRealPath(configFilePath_ + Slash + LastSlash + "Config" + Slash + std::string(CameraConfigurationName)));
    preDefPaths_.push_back(FileFinder::makeRealPath(configFilePath_ + Slash + "sdcard" + Slash + "Orbbec" + Slash + std::string(CameraConfigurationName)));
    for(const auto &val: preDefPaths_) {
        if(FileFinder::isFileExist(val) && doc_->LoadFile(val.c_str()) == 0) {
            rootXMLElement_ = doc_->RootElement();
            if(rootXMLElement_) {
                loadSuccessfully = true;
                configFilePath_  = val;
                // LOG_INFO("loadDefaultConfigPath() configFilePath_=={}", configFilePath_);
                break;
            }
        }
    }
    return loadSuccessfully;
}

bool XmlConfig::loadConfigFile(const std::string &configFilePath) {
    if(configFilePath.empty()) {
        return false;
    }
    std::lock_guard<std::recursive_mutex> lockGuard(mutex_);
    if(FileFinder::isFileExist(configFilePath) && doc_->LoadFile(configFilePath.c_str()) == 0) {
        rootXMLElement_ = doc_->RootElement();
        if(rootXMLElement_) {
            loadFileSuccess_ = true;
            configFilePath_  = configFilePath;
            // LOG_INFO("loadConfigFile() using configFilePath_=={}", configFilePath_);
            return true;
        }
    }
    return false;
}

bool XmlConfig::loadConfigFromBuffer(const char *buffer, size_t size) {
    if(buffer == nullptr || size == 0) {
        return false;
    }
    std::lock_guard<std::recursive_mutex> lockGuard(mutex_);
    if(doc_->Parse(buffer, size) == 0) {
        rootXMLElement_ = doc_->RootElement();
        if(rootXMLElement_) {
            loadFileSuccess_ = true;
            configFilePath_  = "default config!";
            return true;
        }
    }
    return false;
}

#if defined(_WIN32)
bool XmlConfig::loadConfigFile(const std::wstring &configFilePath) {
    if(configFilePath.empty()) {
        return false;
    }
    std::lock_guard<std::recursive_mutex> lockGuard(mutex_);
    if(FileFinder::isFileExist(configFilePath) && doc_->LoadFile(configFilePath.c_str()) == 0) {
        rootXMLElement_ = doc_->RootElement();
        if(rootXMLElement_) {
            loadFileSuccess_ = true;
            configFilePath_  = libobsensor::FileFinder::ws2s(configFilePath);
            return true;
        }
    }
    return false;
}
#endif

std::string XmlConfig::getConfigFilePath() const {
    return configFilePath_;
}

XmlConfig::Ptr XmlConfig::getInstance(std::string configFilePath) {
    if(instancePtr_ == nullptr) {
        std::lock_guard<std::recursive_mutex> lockGuard(mutex_);
        if(instancePtr_ == nullptr) {
            instancePtr_ = std::make_shared<XmlConfig>(configFilePath);
        }
    }
    return instancePtr_;
}

libobsensor::XMLElement *XmlConfig::getRootElement() const {
    if(!loadFileSuccess_) {
        return nullptr;
    }
    return rootXMLElement_;
}
bool XmlConfig::getTextOfLeafNode(const std::string &nodePathName, std::string &text) {
    std::string errorMessage;
    if(nodePathName.empty()) {
        errorMessage = "nodePathName:" + nodePathName + " nodePath is empty!";
        invoke(EventType::ErrorEvent, errorMessage);
        return false;
    }
    auto NodeList = utils::split(nodePathName, ".");
    if(NodeList.empty()) {
        errorMessage = "nodePathName:" + nodePathName + " nodePath is not right!";
        invoke(EventType::ErrorEvent, errorMessage);
        return false;
    }
    std::lock_guard<std::recursive_mutex> lockGuard(mutex_);
    auto                                  rootXMLElement  = rootXMLElement_;
    auto                                  ChildrenElement = rootXMLElement_;
    size_t                                count           = 0;
    for(int i = 0; i < NodeList.size(); ++i) {
        libobsensor::XMLElement *childrenElement = nullptr;
        childrenElement                          = rootXMLElement->FirstChildElement(NodeList[i].c_str());
        if(!childrenElement) {
            errorMessage = "node:" + NodeList[i] + " is not exist!";
            invoke(EventType::ErrorEvent, errorMessage);
            return false;
        }
        count++;
        if(count != NodeList.size()) {
            rootXMLElement = childrenElement;
        }
        if(count == NodeList.size()) {
            ChildrenElement = childrenElement;
        }
    }
    if(ChildrenElement == rootXMLElement_) {
        errorMessage = "nodePathName:" + nodePathName + " cant not be rootElement!";
        invoke(EventType::ErrorEvent, errorMessage);
        return false;
    }
    if(!ChildrenElement->FirstChild() || !ChildrenElement->FirstChild()->ToText()) {
        errorMessage = "nodePathName:" + nodePathName + " is not Text Node or has no message!";
        invoke(EventType::ErrorEvent, errorMessage);
        return false;
    }
    text = ChildrenElement->GetText();
    return true;
}
bool XmlConfig::setTextOfLeafNode(const std::string &nodePathName, const std::string &text) {
    std::string errorMessage;
    if(nodePathName.empty()) {
        errorMessage = "nodePathName:" + nodePathName + " nodePath is empty!";
        invoke(EventType::ErrorEvent, errorMessage);
        return false;
    }
    if(text.empty()) {
        errorMessage = "nodePathName:" + nodePathName + " setValue is empty!";
        invoke(EventType::ErrorEvent, errorMessage);
        return false;
    }
    auto NodeList = utils::split(nodePathName, ".");
    if(NodeList.empty()) {
        errorMessage = "nodePathName:" + nodePathName + " nodePath is not right!";
        invoke(EventType::ErrorEvent, errorMessage);
        return false;
    }
    std::lock_guard<std::recursive_mutex> lockGuard(mutex_);
    auto                                  rootXMLElement  = rootXMLElement_;
    auto                                  ChildrenElement = rootXMLElement_;
    size_t                                count           = 0;
    for(int i = 0; i < NodeList.size(); ++i) {
        libobsensor::XMLElement *childrenElement = nullptr;
        childrenElement                          = rootXMLElement->FirstChildElement(NodeList[i].c_str());
        if(!childrenElement) {
            errorMessage = "node:" + NodeList[i] + " is not exist!";
            invoke(EventType::ErrorEvent, errorMessage);
            return false;
        }
        count++;
        if(count != NodeList.size()) {
            rootXMLElement = childrenElement;
        }
        if(count == NodeList.size()) {
            ChildrenElement = childrenElement;
        }
    }
    if(ChildrenElement == rootXMLElement_) {
        errorMessage = "nodePathName:" + nodePathName + " cant not be rootElement!";
        invoke(EventType::ErrorEvent, errorMessage);
        return false;
    }
    if(!ChildrenElement->FirstChild() || !ChildrenElement->FirstChild()->ToText()) {
        errorMessage = "nodePathName:" + nodePathName + " is not Text Node!";
        invoke(EventType::ErrorEvent, errorMessage);
        return false;
    }
    ChildrenElement->SetText(text.c_str());
    return true;
}
bool XmlConfig::getBooleanValue(const std::string &nodePathName, bool &t) {
    std::string errorMessage;
    if(!loadFileSuccess_) {
        errorMessage = "Default configuration file do not exist,or load failed!";
        invoke(EventType::ErrorEvent, errorMessage);
        return false;
    }
    std::string text;
    bool        status = getTextOfLeafNode(nodePathName, text);
    if(!status) {
        return false;
    }
    status = StringUtils::string2Boolean(text, t);
    if(!status) {
        errorMessage = "text:" + text + " convert Boolean failed! ";
        invoke(EventType::ErrorEvent, errorMessage);
        return false;
    }
    invoke(EventType::ReadEvent, nodePathName);
    return true;
}

bool XmlConfig::isNodeContained(const std::string &nodePathName) {
    std::string errorMessage;
    if(nodePathName.empty()) {
        errorMessage = "nodePathName:" + nodePathName + " nodePath is empty!";
        invoke(EventType::ErrorEvent, errorMessage);
        return false;
    }
    auto NodeList = utils::split(nodePathName, ".");
    if(NodeList.empty()) {
        errorMessage = "nodePathName:" + nodePathName + " nodePath is not right!";
        invoke(EventType::ErrorEvent, errorMessage);
        return false;
    }
    std::lock_guard<std::recursive_mutex> lockGuard(mutex_);
    auto                                  rootXMLElement  = rootXMLElement_;
    auto                                  ChildrenElement = rootXMLElement_;
    size_t                                count           = 0;
    for(int i = 0; i < NodeList.size(); ++i) {
        libobsensor::XMLElement *childrenElement = nullptr;
        childrenElement                          = rootXMLElement->FirstChildElement(NodeList[i].c_str());
        if(!childrenElement) {
            errorMessage = "node:" + NodeList[i] + " is not exist!";
            invoke(EventType::ErrorEvent, errorMessage);
            return false;
        }
        count++;
        if(count != NodeList.size()) {
            rootXMLElement = childrenElement;
        }
        if(count == NodeList.size()) {
            ChildrenElement = childrenElement;
        }
    }
    return true;
}

bool XmlConfig::getIntValue(const std::string &nodePathName, int &t) {
    std::string errorMessage;
    if(!loadFileSuccess_) {
        errorMessage = "Default configuration file do not exist,or load failed!";
        invoke(EventType::ErrorEvent, errorMessage);
        return false;
    }
    std::string text;
    bool        status = getTextOfLeafNode(nodePathName, text);
    if(!status) {
        return false;
    }
    status = StringUtils::string2Int(text, t);
    if(!status) {
        errorMessage = "text:" + text + " convert Int failed! ";
        invoke(EventType::ErrorEvent, errorMessage);
        return false;
    }
    invoke(EventType::ReadEvent, nodePathName);
    return true;
}

bool XmlConfig::getFloatValue(const std::string &nodePathName, float &t) {
    std::string errorMessage;
    if(!loadFileSuccess_) {
        errorMessage = "Default configuration file do not exist,or load failed!";
        invoke(EventType::ErrorEvent, errorMessage);
        return false;
    }
    std::string text;
    bool        status = getTextOfLeafNode(nodePathName, text);
    if(!status) {
        return false;
    }
    status = StringUtils::string2Float(text, t);
    if(!status) {
        errorMessage = "text:" + text + " convert Float failed! ";
        invoke(EventType::ErrorEvent, errorMessage);
        return false;
    }
    invoke(EventType::ReadEvent, nodePathName);
    return true;
}
bool XmlConfig::getDoubleValue(const std::string &nodePathName, double &t) {
    std::string errorMessage;
    if(!loadFileSuccess_) {
        errorMessage = "Default configuration file do not exist,or load failed!";
        invoke(EventType::ErrorEvent, errorMessage);
        return false;
    }
    std::string text;
    bool        status = getTextOfLeafNode(nodePathName, text);
    if(!status) {
        return false;
    }
    status = StringUtils::string2Double(text, t);
    if(!status) {
        errorMessage = "text:" + text + " convert Double failed! ";
        invoke(EventType::ErrorEvent, errorMessage);
        return false;
    }
    invoke(EventType::ReadEvent, nodePathName);
    return true;
}
}  // namespace libobsensor