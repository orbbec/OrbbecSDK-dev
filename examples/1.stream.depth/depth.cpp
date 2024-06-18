// #include <openobsdk/ObSensor.hpp>

// #include "window.hpp"

// int main(int argc, char **argv) try {

//     // Create a pipeline with default device.
//     ob::Pipeline pipe;
//     // By creating config to configure which streams to enable or disable for the pipeline, here the depth stream will be enabled.
//     std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();
//     // Enable depth stream.
//     config->enableVideoStream(OB_STREAM_DEPTH);

//     // Start the pipeline with config.
//     pipe.start(config);
//     // Create a window for rendering, and set the resolution of the window.
//     Window app("DepthViewer", 1280, 720);

//     while(app) {
//         // Wait for up to 100ms for a frameset in blocking mode.
//         auto frameSet = pipe.waitForFrames(100);
//         if(frameSet == nullptr) {
//             continue;
//         }
//         // Get the depth frame from frameset.
//         auto depthFrame = frameSet->depthFrame();

//         // for Y16 format depth frame, print the distance of the center pixel every 30 frames.
//         if(depthFrame->index() % 30 == 0 && depthFrame->format() == OB_FORMAT_Y16) {
//             uint32_t  width  = depthFrame->width();
//             uint32_t  height = depthFrame->height();
//             float     scale  = depthFrame->getValueScale();
//             uint16_t *data   = (uint16_t *)depthFrame->data();

//             // pixel value multiplied by scale is the actual distance value in millimeters.
//             float centerDistance = data[width * height / 2 + width / 2] * scale;

//             // attention: if the distance is 0, it means that the depth camera cannot detect the object (may be out of detection range).
//             std::cout << "Facing an object " << centerDistance << " mm away. " << std::endl;
//         }

//         // Render frame in the window
//         app.addToRender(depthFrame);
//     }

//     // Stop the pipeline
//     pipe.stop();

//     return 0;
// }
// catch(ob::Error &e) {
//     std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
//     exit(EXIT_FAILURE);
// }


#include "opencv2/opencv.hpp"

int main(){
    cv::Mat image;
    return 0;
}