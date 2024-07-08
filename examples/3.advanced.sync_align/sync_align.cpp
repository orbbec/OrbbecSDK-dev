/**
 *Synchronous alignment example
 *
 * In this example, the mirror status of the depth and the color may be inconsistent because the depth or color sensor does not support mirroring.
 * As a result, the images displayed by the depth  and the color are reversed. If this happens, just set the mirror interface to keep the two mirror states
 * consistent.
 * In addition, the resolution obtained by some devices may not support the D2C function, so the resolutions that support D2C, please refer to the
 * product manual.
 * For example: the D2C resolution supported by DaBai DCW is 640x360, but the actual resolution obtained in this example may be 640x480, at this time the user
 * must refer to the product manual to select 640x360 resolution.
 */

#include "utils_opencv.hpp"

#include "libobsensor/ObSensor.hpp"
#include <mutex>
#include <thread>

bool    sync       = false;
float   alpha      = 0.5;
uint8_t align_mode = 0;

// key press event processing
void handleKeyPress(ob_smpl::CVWindow &win, std::shared_ptr<ob::Pipeline> pipe /*, std::shared_ptr<ob::Config> config*/) {
    ////Get the key value
    int key = win.waitKey(10);
    if(key == '+' || key == '=') {
        // Press the + key to increase alpha
        alpha += 0.1f;
        if(alpha >= 1.0f) {
            alpha = 1.0f;
        }
        win.setAlpha(alpha);
    }
    else if(key == '-' || key == '_') {
        // press - key to decrease alpha
        alpha -= 0.1f;
        if(alpha <= 0.0f) {
            alpha = 0.0f;
        }
        win.setAlpha(alpha);
    }
    else if(key == 'F' || key == 'f') {
        // Press the F key to switch synchronization
        sync = !sync;

        if(sync) {
            // enable synchronization
            pipe->enableFrameSync();
        }
        else {
            // turn off sync
            pipe->disableFrameSync();
        }
    }
    else if(key == 't' || key == 'T') {
        // Press the T key to switch align mode
        align_mode = (align_mode + 1) % 2;
    }
}

int main(void) try {
    // Configure which streams to enable or disable for the Pipeline by creating a Config
    auto config = std::make_shared<ob::Config>();

    // enable depth and color streams with specified format
    config->enableVideoStream(OB_STREAM_DEPTH, OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FPS_ANY, OB_FORMAT_Y16);
    config->enableVideoStream(OB_STREAM_COLOR, OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FPS_ANY, OB_FORMAT_RGB);

    // Create a pipeline with default device to manage stream
    auto pipe = std::make_shared<ob::Pipeline>();

    // Start the pipeline with config
    pipe->start(config);

    // Create a Align Filter to align depth frame to color frame
    auto depth2colorAlign = std::make_shared<ob::Align>(OB_STREAM_COLOR);

    // create a filter to align color frame to depth frame
    auto color2depthAlign = std::make_shared<ob::Align>(OB_STREAM_DEPTH);

    // Create a window for rendering and set the resolution of the window
    ob_smpl::CVWindow win("sync_align", 1280, 720, ob_smpl::RENDER_OVERLAY);

    while(win.run()) {
        handleKeyPress(win, pipe);

        auto frameSet = pipe->waitForFrameset(100);
        if(frameSet == nullptr) {
            continue;
        }
        std::shared_ptr<ob::Filter> alignFilter = depth2colorAlign;
        if(align_mode % 2 == 0) {
            alignFilter = color2depthAlign;
        }

        auto alignedFrameSet = alignFilter->process(frameSet);

        // render and display
        win.pushFramesToShow(alignedFrameSet);
    }
    // Stop the Pipeline, no frame data will be generated
    pipe->stop();

    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getFunction() << "\nargs:" << e.getArgs() << "\nmessage:" << e.what() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}
