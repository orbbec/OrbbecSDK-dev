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

#include "libobsensor/OBSensor.hpp"
#include <mutex>
#include <thread>

static bool  sync  = false;
static float alpha = 0.5;

// key press event processing
void keyEventProcess(Window &app, ob::Pipeline &pipe, std::shared_ptr<ob::Config> config) {
    ////Get the key value
    int key = app.waitKey(10);
    if(key == '+' || key == '=') {
        // Press the + key to increase alpha
        alpha += 0.1f;
        if(alpha >= 1.0f) {
            alpha = 1.0f;
        }
        app.setAlpha(alpha);
    }
    else if(key == '-' || key == '_') {
        // press - key to decrease alpha
        alpha -= 0.1f;
        if(alpha <= 0.0f) {
            alpha = 0.0f;
        }
        app.setAlpha(alpha);
    }
    else if(key == 'F' || key == 'f') {
        // Press the F key to switch synchronization
        sync = !sync;
        if(sync) {
            try {
                // enable synchronization
                pipe.enableFrameSync();
            }
            catch(...) {
                std::cerr << "Sync not support" << std::endl;
            }
        }
        else {
            try {
                // turn off sync
                pipe.disableFrameSync();
            }
            catch(...) {
                std::cerr << "Sync not support" << std::endl;
            }
        }
    }
}

int main(void) try {
    // Create a pipeline with default device
    ob::Pipeline pipe;

    // Configure which streams to enable or disable for the Pipeline by creating a Config
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();

    config->enableStream(OB_STREAM_DEPTH);
    config->enableStream(OB_STREAM_COLOR);

    /* Config depth align to color or color align to depth.
    OBStreamType align_to_stream = OB_STREAM_DEPTH; */
    // OBStreamType align_to_stream = OB_STREAM_COLOR;
    auto align = ob::FilterFactory::createFilter("Align");

    // Start the pipeline with config
    pipe.start(config);

    // Create a window for rendering and set the resolution of the window
    Window app("sync_align", 1280, 720, RENDER_OVERLAY);

    while(app) {
        keyEventProcess(app, pipe, config);

        auto frameSet = pipe.waitForFrameset(100);
        if(frameSet == nullptr) {
            continue;
        }
        auto newFrame    = align->process(frameSet);
        auto newFrameSet = newFrame->as<ob::FrameSet>();
        auto colorFrame  = newFrameSet->getFrame(OB_FRAME_COLOR);
        auto depthFrame  = newFrameSet->getFrame(OB_FRAME_DEPTH);
        if(colorFrame == nullptr || depthFrame == nullptr){
            continue;
        }
        app.renderFrameData({ colorFrame, depthFrame });
    }
    // Stop the Pipeline, no frame data will be generated
    pipe.stop();

    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getFunction() << "\nargs:" << e.getArgs() << "\nmessage:" << e.what() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}
