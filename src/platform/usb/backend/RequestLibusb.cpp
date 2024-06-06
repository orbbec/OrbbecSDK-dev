
#include "RequestLibusb.hpp"

#include "logger/Logger.hpp"
#include <thread>

#include "libusb.h"
#include "Request.hpp"

namespace libobsensor {
namespace pal {

void LIBUSB_CALL internalCallback(struct libusb_transfer *transfer) {

    auto urb = reinterpret_cast<UsbRequestLibusb *>(transfer->user_data);
    if(urb) {
        switch(transfer->status) {
        case LIBUSB_TRANSFER_COMPLETED:
            break;
        case LIBUSB_TRANSFER_CANCELLED:
            LOG_DEBUG("transfer cancelled!");
            break;
        case LIBUSB_TRANSFER_ERROR:
            LOG_WARN("Request callback with transfer error!");
            break;
        case LIBUSB_TRANSFER_NO_DEVICE:
            LOG_WARN("Request callback with error: No Device!");
            break;
        default:
            break;
        }

        if(transfer->status != LIBUSB_TRANSFER_CANCELLED) {
            auto response = urb->getShared();
            if(response) {
                auto cb = response->getCallback();
                cb->callback(response);
            }
        }

        urb->setActive(false);
    }
}

UsbRequestLibusb::UsbRequestLibusb(libusb_device_handle *devHandle, std::shared_ptr<UsbEndpoint> endpoint) {
    endpoint_  = endpoint;
    devHandle_ = devHandle;
    if(endpoint_->getType() == OB_USB_ENDPOINT_ISOCHRONOUS) {
        // TODO iso transfer
        isIso_                  = true;
        endpointBytesPerPacket_ = endpoint_->getMaxPacketSize();
        // wMaxPacketSize: [unused:2 (multiplier-1):3 size:11]
        endpointBytesPerPacket_ = (endpointBytesPerPacket_ & 0x07ff) * (((endpointBytesPerPacket_ >> 11) & 3) + 1);
    }
    else {
        isIso_    = false;
        transfer_ = std::shared_ptr<libusb_transfer>(libusb_alloc_transfer(0), [this](libusb_transfer *req) {
            if(!active_)
                libusb_free_transfer(req);
            else
                LOG_ERROR("active request didn't return on time");
        });

        if(endpoint_->getType() == OB_USB_ENDPOINT_INTERRUPT) {

            libusb_fill_interrupt_transfer(transfer_.get(), devHandle, endpoint_->getAddress(), NULL, 0, internalCallback, NULL, 0);
        }
        else {
            libusb_fill_bulk_transfer(transfer_.get(), devHandle, endpoint_->getAddress(), NULL, 0, internalCallback, NULL, 0);
        }
    }

    transfer_->user_data = this;
}

UsbRequestLibusb::~UsbRequestLibusb() noexcept {
    if(active_)
        libusb_cancel_transfer(transfer_.get());

    int attempts = 10;
    while(active_ && attempts--)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

void UsbRequestLibusb::setActive(bool state) {
    active_ = state;
}

int UsbRequestLibusb::getNativeBufferLength() {
    return transfer_->length;
}

void UsbRequestLibusb::setNativeBufferLength(int length) {
    if(isIso_) {
        auto numIsoPacket = (length + endpointBytesPerPacket_ - 1) / endpointBytesPerPacket_;
        /* But keep a reasonable limit: Otherwise we start dropping data */
        if(numIsoPacket > 32)
            numIsoPacket = 32;

        transfer_ = std::shared_ptr<libusb_transfer>(libusb_alloc_transfer(numIsoPacket), [this](libusb_transfer *req) {
            if(!active_)
                libusb_free_transfer(req);
            else
                LOG_ERROR("active request didn't return on time");
        });

        libusb_fill_iso_transfer(transfer_.get(), devHandle_, endpoint_->getAddress(), NULL, 0, 0, internalCallback, NULL, 5000);

        transfer_->length          = numIsoPacket * endpointBytesPerPacket_;
        transfer_->num_iso_packets = numIsoPacket;
        libusb_set_iso_packet_lengths(transfer_.get(), endpointBytesPerPacket_);
    }
    if(endpoint_->getType() == OB_USB_ENDPOINT_INTERRUPT) {

        transfer_->length = endpoint_->getMaxPacketSize();
    }
    if(endpoint_->getType() == OB_USB_ENDPOINT_BULK) {
        transfer_->length = length;
    }
}

int UsbRequestLibusb::getActualLength() const {
    return transfer_->actual_length;
}

void UsbRequestLibusb::setNativeBuffer(uint8_t *buffer) {
    transfer_->buffer = buffer;
}

uint8_t *UsbRequestLibusb::getNativeBuffer() const {
    return transfer_->buffer;
}

void *UsbRequestLibusb::getNativeRequest() const {
    return transfer_.get();
}

usbRequestStatus UsbRequestLibusb::getRequestStatus() const {
    switch(transfer_->status) {
    case LIBUSB_TRANSFER_COMPLETED:
        return OB_USB_TRANSFER_COMPLETED;

    case LIBUSB_TRANSFER_TIMED_OUT:
        return OB_USB_TRANSFER_TIMED_OUT;

    case LIBUSB_TRANSFER_CANCELLED:
        return OB_USB_TRANSFER_CANCELLED;

    case LIBUSB_TRANSFER_ERROR:
        return OB_USB_TRANSFER_ERROR;

    case LIBUSB_TRANSFER_STALL:
        return OB_USB_TRANSFER_STALL;

    case LIBUSB_TRANSFER_NO_DEVICE:
        return OB_USB_TRANSFER_NO_DEVICE;

    case LIBUSB_TRANSFER_OVERFLOW:
        return OB_USB_TRANSFER_OVERFLOW;
    default:
        return OB_USB_TRANSFER_ERROR;
    }
}

std::shared_ptr<UsbRequest> UsbRequestLibusb::getShared() const {
    return shared_.lock();
}

void UsbRequestLibusb::setShared(const std::shared_ptr<UsbRequest> &shared) {
    shared_ = shared;
}
}  // namespace pal
}  // namespace libobsensor
