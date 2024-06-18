// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2020 Orbbec Corporation. All Rights Reserved.

#pragma once

#include "Endpoint.hpp"

#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <vector>

namespace libobsensor {

class UsbRequestCallback;
typedef std::shared_ptr<UsbRequestCallback> obUsbRequestCallback;

class UsbRequest {
public:
    virtual std::shared_ptr<UsbEndpoint> getEndpoint() const                           = 0;
    virtual int                          getActualLength() const                       = 0;
    virtual void                         setCallback(obUsbRequestCallback callback)    = 0;
    virtual obUsbRequestCallback         getCallback() const                           = 0;
    virtual void                         setClientData(void *data)                     = 0;
    virtual void                        *getClientData() const                         = 0;
    virtual void                        *getNativeRequest() const                      = 0;
    virtual const std::vector<uint8_t>  &getBuffer() const                             = 0;
    virtual void                         setBuffer(const std::vector<uint8_t> &buffer) = 0;

    virtual usbRequestStatus getRequestStatus() const = 0;

protected:
    virtual void     setNativeBufferLength(int length) = 0;
    virtual int      getNativeBufferLength()           = 0;
    virtual void     setNativeBuffer(uint8_t *buffer)  = 0;
    virtual uint8_t *getNativeBuffer() const           = 0;
};

typedef std::shared_ptr<UsbRequest> obUsbRequest;

class UsbRequestBase : public UsbRequest {
public:
    virtual std::shared_ptr<UsbEndpoint> getEndpoint() const override {
        return endpoint_;
    }
    virtual void setCallback(obUsbRequestCallback callback) override {
        callback_ = callback;
    }
    virtual obUsbRequestCallback getCallback() const override {
        return callback_;
    }
    virtual void setClientData(void *data) override {
        client_data_ = data;
    }
    virtual void *getClientData() const override {
        return client_data_;
    }
    virtual const std::vector<uint8_t> &getBuffer() const override {
        return buffer_;
    }
    virtual void setBuffer(const std::vector<uint8_t> &buffer) override {
        buffer_ = buffer;
        setNativeBuffer(buffer_.data());
        setNativeBufferLength((int)buffer_.size());
    }

protected:
    void                        *client_data_;
    obUsbRequest                 request;
    std::shared_ptr<UsbEndpoint> endpoint_;
    std::vector<uint8_t>         buffer_;
    obUsbRequestCallback         callback_;
};

class UsbRequestCallback {
    std::function<void(obUsbRequest)> callback_;
    std::mutex                        mutex_;

public:
    UsbRequestCallback(std::function<void(obUsbRequest)> callback) {
        callback_ = callback;
    }

    ~UsbRequestCallback() noexcept {
        cancel();
    }

    void cancel() {
        std::lock_guard<std::mutex> lk(mutex_);
        callback_ = nullptr;
    }

    void callback(obUsbRequest response) {
        std::lock_guard<std::mutex> lk(mutex_);
        if(callback_)
            callback_(response);
    }
};

}  // namespace libobsensor
