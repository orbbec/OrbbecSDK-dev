
extern "C"
{
#include <openobsdk/h/Frame.h>
}

#include <iostream>


void ob_safe_delete_error(ob_error *err)
{
    if (err)
    {
        delete err;
    }
}

int main()
{
    ob_error *err = nullptr;
    auto frame = ob_create_video_frame(OB_FRAME_DEPTH, OB_FORMAT_Y16, 640, 480, OB_DEFAULT_STRIDE_BYTES, &err);
    ob_safe_delete_error(err);
    auto data = ob_frame_get_data(frame, &err);
    ob_safe_delete_error(err);
    std::cout << "Data pointer: " << (uint64_t)data << std::endl;
    auto data_size = ob_frame_get_data_size(frame, &err);
    ob_safe_delete_error(err);
    std::cout << "Data size: " << data_size << std::endl;

    return 0;
}