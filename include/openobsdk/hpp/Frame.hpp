﻿/**
 * @file Frame.hpp
 * @brief Frame related type, which is mainly used to obtain frame data and frame information.
 *
 */
#pragma once

#include "Types.hpp"
#include "openobsdk/h/Frame.h"
#include "openobsdk/hpp/Error.hpp"

#include <memory>
#include <iostream>
#include <typeinfo>

/**
 *  Frame class：
 *         Frame
 *          |
 *      +------+----------+----------+-----------+
 *      |   |     |     |      |
 *   VideoFrame PointsFrame AccelFrame GyroFrame FrameSet
 *     |
 *    +--+------+---------+
 *   |     |     |
 * ColorFrame DepthFrame IRFrame
 *              |
 *           +-----+-----+
 *           |      |
 *        IRLeftFrame IRRightFrame
 */

namespace ob {
class Device;
class Sensor;
class StreamProfile;

/**
 * @brief Define the frame class, which is the base class of all frame types.
 *
 */
class Frame : public std::enable_shared_from_this<Frame> {
protected:
    /**
     * @brief The pointer to the internal (c api level) frame object.
     */
    const ob_frame *impl_;

    /**
     * @brief Construct a new Frame object with a given pointer to the internal frame object.
     *
     * @attention After calling this constructor, the frame object will own the internal frame object, and the internal frame object will be deleted when the
     * frame object is destroyed.
     * @attention The internal frame object should not be deleted by the caller.
     * @attention Please use the FrameFactory to create a Frame object.
     *
     * @param impl The pointer to the internal frame object.
     */
    explicit Frame(const const ob_frame *impl) : impl_(impl) {}

    friend class FrameFactory;

public:
    /**
     * @brief Get the internal (impl) frame object
     *
     * @return const ob_frame* the pointer to the internal frame object.
     */
    const ob_frame *getImpl() const {
        return impl_;
    }

    /**
     * @brief Destroy the Frame object
     */
    virtual ~Frame() noexcept {
        if(impl_) {
            ob_error *error = nullptr;
            ob_delete_frame(impl_, &error);
            Error::handle(&error, false);
            impl_ = nullptr;
        }
    }

    /**
     * @brief Get the type of frame.
     *
     * @return OBFrameType The type of frame.
     */
    virtual OBFrameType getType() const {
        ob_error *error = nullptr;
        auto      type  = ob_frame_get_type(impl_, &error);
        Error::handle(&error);

        return type;
    }

    /**
     * @brief Get the format of the frame.
     *
     * @return OBFormat The format of the frame.
     */
    virtual OBFormat getFormat() const {
        ob_error *error  = nullptr;
        auto      format = ob_frame_get_format(impl_, &error);
        Error::handle(&error);

        return format;
    }

    /**
     * @brief Get the sequence number of the frame.
     *
     * @note The sequence number for each frame is managed by the SDK. It increments by 1 for each frame on each stream.
     *
     * @return uint64_t The sequence number of the frame.
     */
    virtual uint64_t getIndex() const {
        ob_error *error = nullptr;
        auto      index = ob_frame_get_index(impl_, &error);
        Error::handle(&error);

        return index;
    }

    /**
     * @brief Get the timestamp of the frame.
     *
     * @return const uint8_t * The frame data pointer.
     */
    virtual const uint8_t *getData() const {
        ob_error *error = nullptr;
        auto      data  = ob_frame_get_data(impl_, &error);
        Error::handle(&error);

        return data;
    }

    /**
     * @brief Get the size of the frame data.
     *
     * @return uint32_t The size of the frame data.
     * For point cloud data, this returns the number of bytes occupied by all point sets. To find the number of points, divide the dataSize by the structure
     * size of the corresponding point type.
     */
    virtual uint32_t getDataSize() const {
        ob_error *error    = nullptr;
        auto      dataSize = ob_frame_get_data_size(impl_, &error);
        Error::handle(&error);

        return dataSize;
    }

    /**
     * @brief Get the hardware timestamp of the frame in microseconds.
     * @brief The hardware timestamp is the time point when the frame was captured by the device, on device clock domain.
     *
     * @return uint64_t The hardware timestamp of the frame in microseconds.
     */
    uint64_t getTimeStampUs() const {
        ob_error *error       = nullptr;
        auto      timeStampUs = ob_frame_get_timestamp_us(impl_, &error);
        Error::handle(&error);

        return timeStampUs;
    }

    /**
     * @brief Get the system timestamp of the frame in microseconds.
     * @brief The system timestamp is the time point when the frame was received by the host, on host clock domain.
     *
     * @return uint64_t The system timestamp of the frame in microseconds.
     */
    uint64_t getSystemTimeStampUs() const {
        ob_error *error             = nullptr;
        auto      systemTimeStampUs = ob_frame_get_system_timestamp_us(impl_, &error);
        Error::handle(&error);

        return systemTimeStampUs;
    }

    /**
     * @brief Get the global timestamp of the frame in microseconds.
     * @brief The global timestamp is the time point when the frame was was captured by the device, and has been converted to the host clock domain. The
     * conversion process base on the device timestamp and can eliminate the timer drift of the device
     *
     * @attention Only some devices support getting the global timestamp. If the device does not support it, this function will return 0. Check the device
     * support status by @ref Device::isGlobalTimestampSupported() function.
     *
     * @return uint64_t The global timestamp of the frame in microseconds.
     */
    uint64_t getGlobalTimeStampUs() const {
        ob_error *error             = nullptr;
        auto      globalTimeStampUs = ob_frame_get_global_timestamp_us(impl_, &error);
        Error::handle(&error);

        return globalTimeStampUs;
    }

    /**
     * @brief Get the metadata pointer of the frame.
     *
     * @return const uint8_t * The metadata pointer of the frame.
     */
    const uint8_t *getMetadata() const {
        ob_error *error    = nullptr;
        auto      metadata = ob_frame_get_metadata(impl_, &error);
        Error::handle(&error);

        return metadata;
    }

    /**
     * @brief Get the size of the metadata of the frame.
     *
     * @return uint32_t The size of the metadata of the frame.
     */
    uint32_t getMetadataSize() const {
        ob_error *error        = nullptr;
        auto      metadataSize = ob_frame_get_metadata_size(impl_, &error);
        Error::handle(&error);

        return metadataSize;
    }

    /**
     * @brief Check if the frame object has metadata of a given type.
     *
     * @param type The metadata type. refer to @ref OBFrameMetadataType
     * @return bool The result.
     */
    bool hasMetadata(OBFrameMetadataType type) const {
        ob_error *error  = nullptr;
        auto      result = ob_frame_has_metadata(impl_, type, &error);
        Error::handle(&error);

        return result;
    }

    /**
     * @brief Get the metadata value
     *
     * @param type The metadata type. refer to @ref OBFrameMetadataType
     * @return int64_t The metadata value.
     */
    int64_t getMetadataValue(OBFrameMetadataType type) const {
        ob_error *error = nullptr;
        auto      value = ob_frame_get_metadata_value(impl_, type, &error);
        Error::handle(&error);

        return value;
    }

    /**
     * @brief get StreamProfile of the frame
     *
     * @return std::shared_ptr<StreamProfile> The StreamProfile of the frame, may return nullptr if the frame is not captured from a stream.
     */
    std::shared_ptr<const StreamProfile> getStreamProfile() const {
        ob_error *error   = nullptr;
        auto      profile = ob_frame_get_stream_profile(impl_, &error);
        Error::handle(&error);

        // TODO： Implement this function
        throw std::runtime_error("not implemented");
        return nullptr;
    }

    /**
     * @brief get owner sensor of the frame
     *
     * @return std::shared_ptr<Sensor> The owner sensor of the frame, return nullptr if the frame is not owned by any sensor or the sensor is destroyed
     */
    std::shared_ptr<Sensor> getSensor() const {
        ob_error *error  = nullptr;
        auto      sensor = ob_frame_get_sensor(impl_, &error);
        Error::handle(&error);

        // TODO： Implement this function
        throw std::runtime_error("not implemented");
        return nullptr;
    }

    /**
     * @brief get owner device of the frame
     *
     * @return std::shared_ptr<Device> The owner device of the frame, return nullptr if the frame is not owned by any device or the device is destroyed
     */
    std::shared_ptr<Device> getDevice() const {
        ob_error *error  = nullptr;
        auto      device = ob_frame_get_device(impl_, &error);
        Error::handle(&error);

        // TODO： Implement this function
        throw std::runtime_error("not implemented");
        return nullptr;
    }

    /**
     * @brief Clone the frame object.
     * @brief The returned frame object is a new object with the same properties. If copyData is true, the data of the new frame object will be a copy of the
     * data of the original frame object. Otherwise, the data of the new frame object will be randomly generated.
     *
     * @param copyData Whether to copy the data of the original frame object. The default value is true.
     *
     * @return std::shared_ptr<Frame> The new frame object.
     */
    std::shared_ptr<Frame> clone(bool copyData = true) {
        ob_error *error = nullptr;
        auto      impl  = ob_clone_frame(impl_, copyData, &error);
        Error::handle(&error);

        return FrameFactory::createFrameFromImpl(impl);
    }

    /**
     * @brief Check if the runtime type of the frame object is compatible with a given type.
     *
     * @tparam T The given type.
     * @return bool The result.
     */
    template <typename T> bool is();

    /**
     * @brief Convert the frame object to a target type.
     *
     * @tparam T The target type.
     * @return std::shared_ptr<T> The result. If it cannot be converted, an exception will be thrown.
     */
    template <typename T> std::shared_ptr<T> as() {
        if(!is<T>()) {
            throw std::runtime_error("unsupported operation, object's type is not require type");
        }

        return std::make_shared<T>(shared_from_this());
    }
};

/**
 * @brief Define the VideoFrame class, which inherits from the Frame class
 */
class VideoFrame : public Frame {
protected:
    /**
     * @brief Construct a new VideoFrame object with a given pointer to the internal frame object.
     *
     * @attention After calling this constructor, the frame object will own the internal frame object, and the internal frame object will be deleted when the
     * frame object is destroyed.
     * @attention The internal frame object should not be deleted by the caller.
     * @attention Please use the FrameFactory to create a Frame object.
     *
     * @param impl The pointer to the internal frame object.
     */
    explicit VideoFrame(const const ob_frame *impl) : Frame(impl){};
    friend class FrameFactory;

public:
    ~VideoFrame() noexcept override = default;

    /**
     * @brief Get the width of the frame.
     *
     * @return uint32_t The width of the frame.
     */
    uint32_t getWidth() const {
        ob_error *error = nullptr;
        auto      width = ob_video_frame_get_width(impl_, &error);
        Error::handle(&error);

        return width;
    }

    /**
     * @brief Get the height of the frame.
     *
     * @return uint32_t The height of the frame.
     */
    uint32_t getHeight() const {
        ob_error *error  = nullptr;
        auto      height = ob_video_frame_get_height(impl_, &error);
        Error::handle(&error);

        return height;
    }

    /**
     * @brief Get the effective number of pixels in the frame.
     * @attention Only valid for Y8/Y10/Y11/Y12/Y14/Y16 format.
     *
     * @return uint8_t The effective number of pixels in the frame, or 0 if it is an unsupported format.
     */
    uint8_t getPixelAvailableBitSize() const {
        ob_error *error   = nullptr;
        auto      bitSize = ob_video_frame_get_pixel_available_bit_size(impl_, &error);
        Error::handle(&error);

        return bitSize;
    }
};

/**
 * @brief Define the ColorFrame class, which inherits from the VideoFrame classd
 */
class ColorFrame : public VideoFrame {
private:
    /**
     * @brief Construct a new ColorFrame object with a given pointer to the internal frame object.
     *
     * @attention After calling this constructor, the frame object will own the internal frame object, and the internal frame object will be deleted when the
     * frame object is destroyed.
     * @attention The internal frame object should not be deleted by the caller.
     * @attention Please use the FrameFactory to create a Frame object.
     *
     * @param impl The pointer to the internal frame object.
     */
    explicit ColorFrame(const ob_frame *impl) : VideoFrame(impl){};
    friend class FrameFactory;

public:
    ~ColorFrame() noexcept override = default;
};

/**
 * @brief Define the DepthFrame class, which inherits from the VideoFrame class
 */
class DepthFrame : public VideoFrame {
private:
    /**
     * @brief Construct a new DepthFrame object with a given pointer to the internal frame object.
     *
     * @attention After calling this constructor, the frame object will own the internal frame object, and the internal frame object will be deleted when the
     * frame object is destroyed.
     * @attention The internal frame object should not be deleted by the caller.
     * @attention Please use the FrameFactory to create a Frame object.
     *
     * @param impl The pointer to the internal frame object.
     */
    explicit DepthFrame(const ob_frame *impl) : VideoFrame(impl){};
    friend class FrameFactory;

public:
    ~DepthFrame() noexcept override = default;

    /**
     * @brief Get the value scale of the depth frame. The pixel value of depth frame is multiplied by the scale to give a depth value in millimeters.
     *   For example, if valueScale=0.1 and a certain coordinate pixel value is pixelValue=10000, then the depth value = pixelValue*valueScale =
     *   10000*0.1=1000mm.
     *
     * @return float The scale.
     */
    float getValueScale() const {
        ob_error *error = nullptr;
        auto      scale = ob_depth_frame_get_value_scale(impl_, &error);
        Error::handle(&error);

        return scale;
    }
};

/**
 * @brief Define the IRFrame class, which inherits from the VideoFrame class
 *
 */
class IRFrame : public VideoFrame {
private:
    /**
     * @brief Construct a new IRFrame object with a given pointer to the internal frame object.
     *
     * @attention After calling this constructor, the frame object will own the internal frame object, and the internal frame object will be deleted when the
     * frame object is destroyed.
     * @attention The internal frame object should not be deleted by the caller.
     * @attention Please use the FrameFactory to create a Frame object.
     *
     * @param impl The pointer to the internal frame object.
     */
    explicit IRFrame(const ob_frame *impl) : VideoFrame(impl){};
    friend class FrameFactory;

public:
    ~IRFrame() noexcept override = default;
};

/**
 * @brief Define the PointsFrame class, which inherits from the Frame class
 * @brief The PointsFrame class is used to obtain pointcloud data and point cloud information.
 *
 * @note The pointcloud data format can be obtained from the @ref Frame::getFormat() function. Witch can be one of the following formats:
 * - @ref OB_FORMAT_POINT: 32-bit float format with 3D point coordinates (x, y, z), @ref OBPoint
 * - @ref OB_FORMAT_RGB_POINT: 32-bit float format with 3D point coordinates (x, y, z) and point colors (r, g, b) @ref, OBColorPoint
 */
class PointsFrame : public Frame {
private:
    /**
     * @brief Construct a new PointsFrame object with a given pointer to the internal frame object.
     *
     * @attention After calling this constructor, the frame object will own the internal frame object, and the internal frame object will be deleted when the
     * frame object is destroyed.
     * @attention The internal frame object should not be deleted by the caller.
     * @attention Please use the FrameFactory to create a Frame object.
     *
     * @param impl The pointer to the internal frame object.
     */
    explicit PointsFrame(const ob_frame *impl) : Frame(impl){};
    friend class FrameFactory;

public:
    ~PointsFrame() noexcept override = default;

    /**
     * @brief Get the point coordinate value scale of the points frame. The point position value of the points frame is multiplied by the scale to give a
     * position value in millimeters. For example, if scale=0.1, the x-coordinate value of a point is x = 10000, which means that the actual x-coordinate value
     * = x*scale = 10000*0.1 = 1000mm.
     *
     * @return float The coordinate value scale.
     */
    float getCoordinateValueScale() const {
        ob_error *error = nullptr;
        auto      scale = ob_points_frame_get_coordinate_value_scale(impl_, &error);
        Error::handle(&error);

        return scale;
    }
};

/**
 * @brief Define the AccelFrame class, which inherits from the Frame class
 *
 */
class AccelFrame : public Frame {
private:
    explicit AccelFrame(const ob_frame *impl) : Frame(impl){};
    friend class FrameFactory;

public:
    ~AccelFrame() noexcept override = default;

    /**
     * @brief Get the accelerometer frame data
     *
     * @return OBAccelValue The accelerometer frame data
     */
    OBAccelValue getValue() const {
        ob_error *error = nullptr;
        auto      value = ob_accel_frame_get_value(impl_, &error);
        Error::handle(&error);

        return value;
    }

    /**
     * @brief Get the temperature when the frame was sampled
     *
     * @return float The temperature value in celsius
     */
    float getTemperature() const {
        ob_error *error = nullptr;
        auto      temp  = ob_accel_frame_get_temperature(impl_, &error);
        Error::handle(&error);

        return temp;
    }
};

/**
 * @brief Define the GyroFrame class, which inherits from the Frame class
 */
class GyroFrame : public Frame {
private:
    explicit GyroFrame(const ob_frame *impl) : Frame(impl){};
    friend class FrameFactory;

public:
    ~GyroFrame() noexcept override = default;

    /**
     * @brief Get the gyro frame data
     *
     * @return OBAccelValue The gyro frame data
     */
    OBGyroValue getValue() const {
        ob_error *error = nullptr;
        auto      value = ob_gyro_frame_get_value(impl_, &error);
        Error::handle(&error);

        return value;
    }

    /**
     * @brief Get the temperature when the frame was sampled
     *
     * @return float The temperature value in celsius
     */
    float getTemperature() const {
        ob_error *error = nullptr;
        auto      temp  = ob_gyro_frame_get_temperature(impl_, &error);
        Error::handle(&error);

        return temp;
    }
};

/**
 * @brief Define the FrameSet class, which inherits from the Frame class
 * @brief A FrameSet is a container for multiple frames of different types.
 */
class FrameSet : public Frame {
private:
    explicit FrameSet(const ob_frame *impl) : Frame(impl){};
    friend class FrameFactory;

public:
    ~FrameSet() noexcept override;

    /**
     * @brief Get the number of frames in the FrameSet
     *
     * @return uint32_t The number of frames
     */
    uint32_t getFrameCount() const {
        ob_error *error = nullptr;
        auto      count = ob_frameset_get_frame_count(impl_, &error);
        Error::handle(&error);

        return count;
    }

    /**
     * @brief Get a frame of a specific type from the FrameSet
     *
     * @param frameType The type of sensor
     * @return std::shared_ptr<Frame> The corresponding type of frame
     */
    std::shared_ptr<const Frame> getFrame(OBFrameType frameType) const {
        ob_error *error = nullptr;
        auto      frame = ob_frameset_get_frame(impl_, frameType, &error);
        Error::handle(&error);

        return FrameFactory::createFrameFromImpl(frame);
    }

    /**
     * @brief Get a frame at a specific index from the FrameSet
     *
     * @param index The index of the frame
     * @return std::shared_ptr<Frame> The frame at the specified index
     */
    std::shared_ptr<const Frame> getFrame(int index) const {
        ob_error *error = nullptr;
        auto      frame = ob_frameset_get_frame_by_index(impl_, index, &error);
        Error::handle(&error);

        return FrameFactory::createFrameFromImpl(frame);
    }

    /**
     * @brief Push a frame to the FrameSet
     *
     * @attention If the FrameSet contains the same type of frame, the new frame will replace the old one.
     *
     * @param frame The frame to be pushed
     */
    void pushFrame(std::shared_ptr<const Frame> frame) {
        ob_error *error = nullptr;

        // unsafe operation, need to cast const to non-const
        auto unConstImpl = const_cast<ob_frame *>(impl_);

        auto otherImpl = frame->getImpl();
        ob_frameset_push_frame(unConstImpl, otherImpl, &error);

        Error::handle(&error);
    }
};

/**
 * @brief FrameFactory class, which provides some static functions to create frame objects
 */
class FrameFactory {
public:
    /**
     * @brief Create a Frame object from a given pointer to the internal frame object (impl).
     * @brief This function will create a Frame object of the corresponding type based on the frame type.
     *
     * @attention After calling this function, the return frame object will own the internal frame object, and the internal frame object will be deleted when
     * the frame object is destroyed.
     * @attention The internal frame object should not be deleted by the caller.
     *
     * @param impl The pointer to the internal frame object.
     */
    static std::shared_ptr<Frame> createFrameFromImpl(const ob_frame *impl) {
        if(impl == nullptr) {
            return nullptr;
        }
        ob_error *error     = nullptr;
        auto      frameType = ob_frame_get_type(impl, &error);
        Error::handle(&error);

        switch(frameType) {
        case OB_FRAME_IR_LEFT:   // Follow
        case OB_FRAME_IR_RIGHT:  // Follow
        case OB_FRAME_IR:
            return std::shared_ptr<Frame>(new IRFrame(impl));
        case OB_FRAME_DEPTH:
            return std::shared_ptr<Frame>(new DepthFrame(impl));
        case OB_FRAME_COLOR:
            return std::shared_ptr<Frame>(new ColorFrame(impl));
        case OB_FRAME_GYRO:
            return std::shared_ptr<Frame>(new GyroFrame(impl));
        case OB_FRAME_ACCEL:
            return std::shared_ptr<Frame>(new AccelFrame(impl));
        case OB_FRAME_POINTS:
            return std::shared_ptr<Frame>(new PointsFrame(impl));
        case OB_FRAME_SET:
            return std::shared_ptr<Frame>(new FrameSet(impl));
        default:
            std::cout << "ob::FrameFactory:: createFrameFromImpl() did not catch frame type: " << frameType << std::endl;
            break;
        }
        return nullptr;
    }

    /**
     * @brief Create a Frame object of a specific type with a given format and data size.
     *
     * @param frameType The type of the frame.
     * @param format The format of the frame.
     * @param dataSize The size of the data in bytes.
     * @return std::shared_ptr<Frame> The created frame object.
     */
    static std::shared_ptr<Frame> createFrame(OBFrameType frameType, OBFormat format, uint32_t dataSize) {
        ob_error *error = nullptr;
        auto      impl  = ob_create_frame(frameType, format, dataSize, &error);
        Error::handle(&error);

        return createFrameFromImpl(impl);
    }

    /**
     * @brief Create a VideoFrame object of a specific type with a given format, width, height, and stride.
     * @note If stride is not specified, it will be calculated based on the width and format.
     *
     * @param frameType The type of the frame.
     * @param format The format of the frame.
     * @param width The width of the frame.
     * @param height The height of the frame.
     * @param stride The stride of the frame.
     *
     * @return std::shared_ptr<VideoFrame> The created video frame object.
     */
    static std::shared_ptr<VideoFrame> createVideoFrame(OBFrameType frameType, OBFormat format, uint32_t width, uint32_t height, uint32_t stride = 0) {
        ob_error *error = nullptr;
        auto      impl  = ob_create_video_frame(frameType, format, width, height, stride, &error);
        Error::handle(&error);

        auto frame = createFrameFromImpl(impl);
        return frame->as<VideoFrame>();
    }

    /**
     * @brief Create a Frame From (according to)Stream Profile object
     *
     * @param profile The stream profile object to create the frame from.
     *
     * @return std::shared_ptr<Frame>  The created frame object.
     */
    static std::shared_ptr<Frame> createFrameFromStreamProfile(std::shared_ptr<const StreamProfile> profile) {
        ob_error *error = nullptr;
        auto      impl  = ob_create_frame_from_stream_profile(profile->getImpl(), &error);
        Error::handle(&error);

        return createFrameFromImpl(impl);
    }

    /**
     * @brief The callback function to destroy the buffer when the frame is destroyed.
     */
    typedef std::function<void(uint8_t *)> BufferDestroyCallback;

    /**
     * @brief Create a frame object based on an externally created buffer.
     *
     * @attention The buffer is owned by the caller, and will not be destroyed by the frame object. The user should ensure that the buffer is valid and not
     * modified.
     *
     * @param[in] frame_type Frame object type.
     * @param[in] format Frame object format.
     * @param[in] buffer Frame object buffer.
     * @param[in] buffer_size Frame object buffer size.
     * @param[in] destroyCallback Destroy callback, will be called when the frame object is destroyed.
     *
     * @return std::shared_ptr<Frame> The created frame object.
     */
    static std::shared_ptr<Frame> createFrameFromBuffer(OBFrameType frameType, OBFormat format, uint8_t *buffer, BufferDestroyCallback destroyCallback,
                                                        uint32_t bufferSize) {
        ob_error *error = nullptr;
        auto      ctx   = new BufferDestroyContext{ destroyCallback };
        auto      impl  = ob_create_frame_from_buffer(frameType, format, buffer, bufferSize, &FrameFactory::BufferDestroy, ctx, &error);
        Error::handle(&error);

        return createFrameFromImpl(impl);
    }

    /**
     * @brief Create a video frame object based on an externally created buffer.
     *
     * @attention The buffer is owned by the user and will not be destroyed by the frame object. The user should ensure that the buffer is valid and not
     * modified.
     * @attention The frame object is created with a reference count of 1, and the reference count should be decreased by calling @ref ob_delete_frame() when it
     * is no longer needed.
     *
     * @param[in] frameType Frame object type.
     * @param[in] format Frame object format.
     * @param[in] width Frame object width.
     * @param[in] height Frame object height.
     * @param[in] buffer Frame object buffer.
     * @param[in] bufferSize Frame object buffer size.
     * @param[in] destroyCallback Destroy callback, will be called when the frame object is destroyed.
     * @param[in] stride Row span in bytes. If 0, the stride is calculated based on the width and format.
     *
     * @return std::shared_ptr<VideoFrame> The created video frame object.
     */
    static std::shared_ptr<VideoFrame> createVideoFrameFromBuffer(OBFrameType frameType, OBFormat format, uint32_t width, uint32_t height, uint8_t *buffer,
                                                                  BufferDestroyCallback destroyCallback, uint32_t bufferSize, uint32_t stride = 0) {
        ob_error *error = nullptr;
        auto      ctx   = new BufferDestroyContext{ destroyCallback };
        auto impl = ob_create_video_frame_from_buffer(frameType, format, width, height, stride, buffer, bufferSize, &FrameFactory::BufferDestroy, ctx, &error);
        Error::handle(&error);

        auto frame = createFrameFromImpl(impl);
        return frame->as<VideoFrame>();
    }

private:
    struct BufferDestroyContext {
        BufferDestroyCallback callback;
    };

    static void BufferDestroy(uint8_t *buffer, void *context) {
        BufferDestroyContext *ctx = static_cast<BufferDestroyContext *>(context);
        ctx->callback(buffer);
        delete ctx;
    }
};

// Define the is() template function for the Frame class
template <typename T> bool Frame::is() {
    switch(this->type()) {
    case OB_FRAME_IR_LEFT:   // Follow
    case OB_FRAME_IR_RIGHT:  // Follow
    case OB_FRAME_IR:
        return (typeid(T) == typeid(IRFrame) || typeid(T) == typeid(VideoFrame));
    case OB_FRAME_DEPTH:
        return (typeid(T) == typeid(DepthFrame) || typeid(T) == typeid(VideoFrame));
    case OB_FRAME_COLOR:
        return (typeid(T) == typeid(ColorFrame) || typeid(T) == typeid(VideoFrame));
    case OB_FRAME_GYRO:
        return (typeid(T) == typeid(GyroFrame));
    case OB_FRAME_ACCEL:
        return (typeid(T) == typeid(AccelFrame));
    case OB_FRAME_POINTS:
        return (typeid(T) == typeid(PointsFrame));
    case OB_FRAME_SET:
        return (typeid(T) == typeid(FrameSet));
    default:
        std::cout << "ob::Frame::is() did not catch frame type: " << (int)this->type() << std::endl;
        break;
    }
    return false;
}
}  // namespace ob
