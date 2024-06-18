
#pragma once
#include <libusb.h>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace libobsensor {

class UsbContext {
public:
    UsbContext();
    ~UsbContext() noexcept;

    libusb_context *get();

    void startEventHandler();
    void stopEventHandler();

    size_t         deviceCount();
    libusb_device *getDevice(uint8_t index);
    void           refreshDeviceList();

private:
    libusb_context *ctx_;

    size_t          count_;
    libusb_device **list_;

    std::mutex  mutex_;
    int         handlerRequests_   = 0;
    int         killHandlerThread_ = 0;
    std::thread eventHandler_;
    // todo-lingyi
    // std::vector<std::thread> eventHandlerList_;
};

}  // namespace libobsensor
