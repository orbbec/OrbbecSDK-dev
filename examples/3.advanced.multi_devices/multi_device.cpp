#include "libobsensor/ObSensor.hpp"

#include "utils_opencv.hpp"

#include <mutex>

const int maxDeviceCount = 9;

std::vector<std::shared_ptr<const ob::Frame>> frames;
std::shared_ptr<ob::Frame>                    colorFrames[maxDeviceCount];
std::shared_ptr<ob::Frame>                    depthFrames[maxDeviceCount];
std::shared_ptr<ob::Frame>                    irFrames[maxDeviceCount];
std::mutex                                    frameMutex;

void StartStream(std::vector<std::shared_ptr<ob::Pipeline>> pipes);
void StopStream(std::vector<std::shared_ptr<ob::Pipeline>> pipes);

int main() try {
    // Create a Context
    ob::Context ctx;

    // Query the list of connected devices
    auto devList = ctx.queryDeviceList();
    // Get the number of connected devices
    int devCount = devList->getCount();

    // traverse the device list and create a pipe
    std::vector<std::shared_ptr<ob::Pipeline>> pipes;
    for(int i = 0; i < devCount; i++) {
        // Get the device and create the pipeline
        auto dev  = devList->getDevice(i);
        auto pipe = std::make_shared<ob::Pipeline>(dev);
        pipes.push_back(pipe);
    }

    // Open depth and color streams for all devices
    StartStream(pipes);

    // Create a window for rendering and set the resolution of the window
    ob_smpl::CVWindow win("MultiDeviceViewer", 1280, 720, ob_smpl::RENDER_GRID);

    while(win.run()) {
        // Render a set of frame in the window, here will render the depth, color or infrared frames of all devices, ob_smpl::RENDER_GRID
        // means that all frames will be rendered in a grid arrangement
        frames.clear();
        {
            std::lock_guard<std::mutex> lock(frameMutex);
            int                         i = 0;
            for(auto pipe: pipes) {
                if(colorFrames[i] != nullptr) {
                    frames.emplace_back(colorFrames[i]);
                }

                if(depthFrames[i] != nullptr) {
                    frames.emplace_back(depthFrames[i]);
                }

                if(irFrames[i] != nullptr) {
                    frames.emplace_back(irFrames[i]);
                }
                i++;
            }
        }
        if(frames.size() > 0){
            win.renderFrame(frames);
        }
    }

    frames.clear();
    StopStream(pipes);

    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getFunction() << "\nargs:" << e.getArgs() << "\nmessage:" << e.what() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}

void StartStream(std::vector<std::shared_ptr<ob::Pipeline>> pipes) {
    int i = 0;
    for(auto &&pipe: pipes) {
        std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
        // Get the depth camera configuration list
        auto                                    depthProfileList = pipe->getStreamProfileList(OB_SENSOR_DEPTH);
        std::shared_ptr<ob::VideoStreamProfile> depthProfile     = nullptr;
        if(depthProfileList) {
            // Open the default profile of Depth Sensor, which can be configured through the configuration file
            depthProfile = std::const_pointer_cast<ob::StreamProfile>(depthProfileList->getProfile(OB_PROFILE_DEFAULT))->as<ob::VideoStreamProfile>();
        }
        config->enableStream(depthProfile);

        // Get the color camera configuration list
        try {
            auto                                    colorProfileList = pipe->getStreamProfileList(OB_SENSOR_COLOR);
            std::shared_ptr<ob::VideoStreamProfile> colorProfile     = nullptr;
            if(colorProfileList) {
                // Open the default profile of Color Sensor, which can be configured through the configuration file
                colorProfile = std::const_pointer_cast<ob::StreamProfile>(colorProfileList->getProfile(OB_PROFILE_DEFAULT))->as<ob::VideoStreamProfile>();
            }
            config->enableStream(colorProfile);
        }
        catch(ob::Error &e) {
            std::cerr << "function:" << e.getFunction() << "\nargs:" << e.getArgs() << "\nmessage:" << e.what() << "\ntype:" << e.getExceptionType()
                      << std::endl;
            exit(EXIT_FAILURE);
        }

        // Start the pipeline and pass in the configuration
        pipe->start(config, [i](std::shared_ptr<ob::FrameSet> frameSet) {
            std::lock_guard<std::mutex> lock(frameMutex);
            auto                        colorFrameRaw = frameSet->getFrame(OB_FRAME_COLOR);
            auto                        depthFrameRaw = frameSet->getFrame(OB_FRAME_DEPTH);
            if(colorFrameRaw && colorFrameRaw) {
                auto colorFrame = colorFrameRaw->as<ob::VideoFrame>();
                auto depthFrame = depthFrameRaw->as<ob::VideoFrame>();
                colorFrames[i]  = colorFrame;
                depthFrames[i]  = depthFrame;
            }
        });
        i++;
    }
}

void StopStream(std::vector<std::shared_ptr<ob::Pipeline>> pipes) {
    int i = 0;
    for(auto &&pipe: pipes) {
        if(colorFrames[i])
            colorFrames->reset();
        if(depthFrames[i])
            depthFrames->reset();
        if(irFrames[i])
            irFrames->reset();
        // stop the pipeline
        pipe->stop();
        i++;
    }
}
