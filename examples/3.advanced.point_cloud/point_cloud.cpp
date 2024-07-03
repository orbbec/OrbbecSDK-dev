#include <libobsensor/ObSensor.hpp>

#include <fstream>
#include <iostream>

#include "utils.hpp"

#define KEY_ESC 27
#define KEY_R 82
#define KEY_r 114

// Save point cloud data to ply
void savePointsToPly(std::shared_ptr<ob::Frame> frame, std::string fileName) {
    int   pointsSize = frame->getDataSize() / sizeof(OBPoint);
    FILE *fp         = fopen(fileName.c_str(), "wb+");
    fprintf(fp, "ply\n");
    fprintf(fp, "format ascii 1.0\n");
    fprintf(fp, "element vertex %d\n", pointsSize);
    fprintf(fp, "property float x\n");
    fprintf(fp, "property float y\n");
    fprintf(fp, "property float z\n");
    fprintf(fp, "end_header\n");

    OBPoint *point = (OBPoint *)frame->getData();
    for(int i = 0; i < pointsSize; i++) {
        fprintf(fp, "%.3f %.3f %.3f\n", point->x, point->y, point->z);
        point++;
    }

    fflush(fp);
    fclose(fp);
}

// Save colored point cloud data to ply
void saveRGBPointsToPly(std::shared_ptr<ob::Frame> frame, std::string fileName) {
    int   pointsSize = frame->getDataSize() / sizeof(OBColorPoint);
    FILE *fp         = fopen(fileName.c_str(), "wb+");
    fprintf(fp, "ply\n");
    fprintf(fp, "format ascii 1.0\n");
    fprintf(fp, "element vertex %d\n", pointsSize);
    fprintf(fp, "property float x\n");
    fprintf(fp, "property float y\n");
    fprintf(fp, "property float z\n");
    fprintf(fp, "property uchar red\n");
    fprintf(fp, "property uchar green\n");
    fprintf(fp, "property uchar blue\n");
    fprintf(fp, "end_header\n");

    OBColorPoint *point = (OBColorPoint *)frame->getData();
    for(int i = 0; i < pointsSize; i++) {
        fprintf(fp, "%.3f %.3f %.3f %d %d %d\n", point->x, point->y, point->z, (int)point->r, (int)point->g, (int)point->b);
        point++;
    }

    fflush(fp);
    fclose(fp);
}

int main(int argc, char **argv) try {
    // create pipeline
    ob::Pipeline pipeline;

    // Enable frame synchronization to ensure depth frame and color frame on output frameset are synchronized.
    pipeline.enableFrameSync();

    // start pipeline with config
    pipeline.start();

    // Create a point cloud Filter object (the device parameters will be obtained inside the Pipeline when the point cloud filter is created, so try to
    // configure the device before creating the filter)
    ob::PointCloudFilter pointCloud;

    // operation prompt
    std::cout << "Press R or r to create RGBD PointCloud and save to ply file! " << std::endl;
    std::cout << "Press D or d to create Depth PointCloud and save to ply file! " << std::endl;
    std::cout << "Press ESC to exit! " << std::endl;

    int count = 0;
    while(true) {
        auto frameset = pipeline.waitForFrameset(100);
        if(kbhit()) {
            int key = getch();
            // Press the ESC key to exit
            if(key == KEY_ESC) {
                break;
            }
            if(key == 'R' || key == 'r') {
                count = 0;
                // Limit up to 10 repetitions
                while(count++ < 10) {
                    // Wait for a frame of data, the timeout is 100ms
                    auto frameset = pipeline.waitForFrameset(100);
                    if(frameset != nullptr && frameset->getFrame(OB_FRAME_COLOR)->as<ob::ColorFrame>() != nullptr && frameset->getFrame(OB_FRAME_DEPTH)->as<ob::DepthFrame>() != nullptr) {
                        // point position value multiply depth value scale to convert uint to millimeter (for some devices, the default depth value uint is not
                        // millimeter)
                        auto depthValueScale = frameset->getFrame(OB_FRAME_DEPTH)->as<ob::DepthFrame>()->getValueScale();
                        pointCloud.setPositionDataScaled(depthValueScale);
                        try {
                            // Generate a colored point cloud and save it
                            std::cout << "Save RGBD PointCloud ply file..." << std::endl;
                            pointCloud.setCreatePointFormat(OB_FORMAT_RGB_POINT);
                            std::shared_ptr<ob::Frame> frame = pointCloud.process(frameset);
                            saveRGBPointsToPly(frame, "RGBPoints.ply");
                            std::cout << "RGBPoints.ply Saved" << std::endl;
                        }
                        catch(std::exception &e) {
                            std::cout << "Get point cloud failed" << std::endl;
                        }
                        break;
                    }
                    else {
                        std::cout << "Get color frame or depth frame failed!" << std::endl;
                    }
                }
            }
            else if(key == 'D' || key == 'd') {
                count = 0;
                // Limit up to 10 repetitions
                while(count++ < 10) {
                    // Wait for up to 100ms for a frameset in blocking mode.
                    auto frameset = pipeline.waitForFrameset(100);
                    if(frameset != nullptr && frameset->getFrame(OB_FRAME_DEPTH)->as<ob::DepthFrame>() != nullptr) {
                        // point position value multiply depth value scale to convert uint to millimeter (for some devices, the default depth value uint is not
                        // millimeter)
                        auto depthValueScale = frameset->getFrame(OB_FRAME_DEPTH)->as<ob::DepthFrame>()->getValueScale();
                        pointCloud.setPositionDataScaled(depthValueScale);
                        try {
                            // generate point cloud and save
                            std::cout << "Save Depth PointCloud to ply file..." << std::endl;
                            pointCloud.setCreatePointFormat(OB_FORMAT_POINT);
                            std::shared_ptr<ob::Frame> frame = pointCloud.process(frameset);
                            savePointsToPly(frame, "DepthPoints.ply");
                            std::cout << "DepthPoints.ply Saved" << std::endl;
                        }
                        catch(std::exception &e) {
                            std::cout << "Get point cloud failed" << std::endl;
                        }
                        break;
                    }
                }
            }
        }
    }
    // stop the pipeline
    pipeline.stop();

    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getFunction() << "\nargs:" << e.getArgs() << "\nmessage:" << e.what() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}
