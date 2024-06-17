

#include "DeviceLibusb.hpp"
#include "exception/ObException.hpp"

#include <map>
#include <mutex>
#include <algorithm>

#ifdef __ANDROID__
#include "core/Context.hpp"
#include "pal/android/AndroidUsbDeviceManager.hpp"
#include "pal/android/AndroidPal.hpp"
#endif

namespace libobsensor {

#ifdef __ANDROID__
UsbDeviceLibusb::UsbDeviceLibusb(std::shared_ptr<AndroidUsbDeviceManager> usbManager, libusb_device *device, libusb_device_handle *deviceHandle,
                                 libusb_device_descriptor &desc, std::shared_ptr<UsbContext> context, std::string devUrl)
    : device_(device), usbDeviceDescriptor_(desc), context_(context), handle_(deviceHandle), devUrl_(devUrl), androidUsbManager_(usbManager) {
    init();
}

UsbDeviceLibusb::~UsbDeviceLibusb() noexcept {
    androidUsbManager_->closeUsbDevice(devUrl_);
    androidUsbManager_ = nullptr;
}
#else
typedef struct {
    libusb_device_handle *handle;
    uint32_t              use_count;
} device_handle_ref;

static std::map<std::string, device_handle_ref> deviceHandleRefList;
static std::mutex                               dev_handle_mutex;

std::string getDevicePath(libusb_device *usbDevice) {
#ifdef WIN32
    auto path = libusb_get_windows_path(usbDevice);
    if(!path) {
        return "";
    }
    return path;
#else
    auto usb_bus = std::to_string(libusb_get_bus_number(usbDevice));

    // As per the USB 3.0 specs, the current maximum limit for the depth is 7.
    const auto               max_usb_depth            = 8;
    uint8_t                  usb_ports[max_usb_depth] = {};
    std::stringstream        port_path;
    auto                     port_count = libusb_get_port_numbers(usbDevice, usb_ports, max_usb_depth);
    auto                     usb_dev    = std::to_string(libusb_get_device_address(usbDevice));
    libusb_device_descriptor dev_desc{};
    auto                     r = libusb_get_device_descriptor(usbDevice, &dev_desc);
    (void)r;

    for(int i = 0; i < port_count; ++i) {
        port_path << std::to_string(usb_ports[i]) << (((i + 1) < port_count) ? "." : "");
    }

    return usb_bus + "-" + port_path.str() + "-" + usb_dev;
#endif
}

int safe_open_device(libusb_device *device, libusb_device_handle **dev_handle) {
    std::unique_lock<std::mutex> lock(dev_handle_mutex);
    std::string                  devUrl = getDevicePath(device);
    int                          sts;
    auto                         devHandleRefIter = deviceHandleRefList.find(devUrl);
    if(devHandleRefIter != deviceHandleRefList.end()) {
        *dev_handle = devHandleRefIter->second.handle;
        devHandleRefIter->second.use_count++;
        sts = LIBUSB_SUCCESS;
        // LOG_ERROR("safe_open_device:" << devUrl << ", add ref";
    }
    else {
        sts = libusb_open(device, dev_handle);
        if(sts == LIBUSB_SUCCESS) {
            libusb_ref_device(device);
            deviceHandleRefList.insert({ devUrl, { *dev_handle, 1 } });
            // LOG_ERROR("safe_open_device:" << devUrl << ", open";
        }
    }
    return sts;
}

int safe_close_device(libusb_device *device, libusb_device_handle *dev_handle) {
    std::unique_lock<std::mutex> lock(dev_handle_mutex);
    std::string                  devUrl           = getDevicePath(device);
    auto                         devHandleRefIter = deviceHandleRefList.find(devUrl);
    if(devHandleRefIter != deviceHandleRefList.end()) {
        devHandleRefIter->second.use_count--;
        if(devHandleRefIter->second.use_count == 0) {
            deviceHandleRefList.erase(devHandleRefIter);
            libusb_close(dev_handle);
            libusb_unref_device(device);
        }
    }
    return LIBUSB_SUCCESS;
}

UsbDeviceLibusb::UsbDeviceLibusb(libusb_device *device, const libusb_device_descriptor &desc, std::shared_ptr<UsbContext> context)
    : device_(device), usbDeviceDescriptor_(desc), context_(context) {

    handle_  = NULL;
    auto sts = safe_open_device(device, &handle_);
    if(sts != LIBUSB_SUCCESS) {
        auto              rs_sts = libusbStatusToOb(sts);
        std::stringstream msg;
        msg << "failed to open usb device! " << " error: " << usbStatusToString.at(rs_sts);
        LOG_ERROR(msg.str());
        throw std::runtime_error(msg.str());
    }
    init();
}

UsbDeviceLibusb::~UsbDeviceLibusb() noexcept {
    safe_close_device(device_, handle_);
}
#endif

void UsbDeviceLibusb::init() {
    UsbDescriptor ud = { usbDeviceDescriptor_.bLength, usbDeviceDescriptor_.bDescriptorType, std::vector<uint8_t>(usbDeviceDescriptor_.bLength) };
    memcpy(ud.data.data(), &usbDeviceDescriptor_, usbDeviceDescriptor_.bLength);
    descriptors_.push_back(ud);

    for(uint8_t c = 0; c < usbDeviceDescriptor_.bNumConfigurations; ++c) {
        libusb_config_descriptor *config{};
        auto                      ret = libusb_get_config_descriptor(device_, c, &config);
        if(LIBUSB_SUCCESS != ret) {
            LOG_WARN("failed to read USB config descriptor: error={}", ret);
            continue;
        }

        std::shared_ptr<UsbInterfaceLibusb> curr_ctrl_intf;
        for(uint8_t i = 0; i < config->bNumInterfaces; ++i) {
            auto inf      = config->interface[i];
            auto curr_inf = std::make_shared<UsbInterfaceLibusb>(inf);
            interfaces_.push_back(curr_inf);
            switch(inf.altsetting->bInterfaceClass) {
            case OB_USB_CLASS_VIDEO: {
                if(inf.altsetting->bInterfaceSubClass == OB_USB_SUBCLASS_VIDEO_CONTROL)
                    curr_ctrl_intf = curr_inf;
                if(inf.altsetting->bInterfaceSubClass == OB_USB_SUBCLASS_VIDEO_STREAMING)
                    curr_ctrl_intf->addAssociatedInterface(curr_inf);
                break;
            }
            default:
                break;
            }
            for(int j = 0; j < inf.num_altsetting; j++) {
                auto          d     = inf.altsetting[j];
                UsbDescriptor altUd = { d.bLength, d.bDescriptorType, std::vector<uint8_t>(d.bLength) };
                memcpy(altUd.data.data(), &d, d.bLength);
                descriptors_.push_back(altUd);
                for(int k = 0; k < d.extra_length;) {
                    auto          l       = d.extra[k];
                    auto          dt      = d.extra[k + 1];
                    UsbDescriptor extraUd = { l, dt, std::vector<uint8_t>(l) };
                    memcpy(extraUd.data.data(), &d.extra[k], l);
                    descriptors_.push_back(extraUd);
                    k += l;
                }
            }
        }

        libusb_free_config_descriptor(config);
    }
}

const std::shared_ptr<UsbInterface> UsbDeviceLibusb::getInterface(uint8_t interface_number) const {
    auto it = std::find_if(interfaces_.begin(), interfaces_.end(),
                           [interface_number](const std::shared_ptr<UsbInterface> &i) { return interface_number == i->getNumber(); });
    if(it == interfaces_.end())
        return nullptr;
    return *it;
}

std::shared_ptr<HandleLibusb> UsbDeviceLibusb::get_handle(uint8_t interface_number) {
    // BEGIN_TRY_EXECUTE({
        auto i = getInterface(interface_number);
        if(!i)
            return nullptr;
        auto intf = std::dynamic_pointer_cast<UsbInterfaceLibusb>(i);
        return std::make_shared<HandleLibusb>(context_, handle_, intf);
    // })
    // CATCH_EXCEPTION_AND_EXECUTE(return nullptr)
}

const std::shared_ptr<UsbMessenger> UsbDeviceLibusb::open(uint8_t interface_number) {
    auto h = get_handle(interface_number);
    if(!h)
        return nullptr;
    return std::make_shared<UsbMessengerLibusb>(shared_from_this(), h);
}

}  // namespace libobsensor
