#include "AlignImpl.hpp"

#include <iostream>
#include <fstream>
#include <regex>

int parse_obviewer_data(char *filename, OBCameraIntrinsic &depth_intr, OBCameraIntrinsic &color_intr, OBCameraDistortion &depth_disto,
                        OBCameraDistortion &color_disto, OBTransform &trans) {
    std::ifstream file(filename);
    if(!file.is_open()) {
        std::cerr << "Error opening file." << std::endl;
        return 1;
    }

    std::string line;

    const std::string numberPattern = R"([+-]?\d+\.?\d*[eE]?[+-]?\d*)";

    // Define regular expressions for each type of line
    std::regex extrinsicRotationRegex(
        R"(rotation:(-?\d+\.\d+) (-?\d+\.\d+) (-?\d+\.\d+) (-?\d+\.\d+) (-?\d+\.\d+) (-?\d+\.\d+) (-?\d+\.\d+) (-?\d+\.\d+) (-?\d+\.\d+))");
    // Define regular expressions for each type of line using the number pattern
    std::regex intrinsicRegex(R"(fx:()" + numberPattern + R"() fy:()" + numberPattern + R"() cx:()" + numberPattern + R"() cy:()" + numberPattern
                              + R"() width:(\d+) height:(\d+))");

    std::regex distortionRegex(R"(k1:()" + numberPattern + R"() k2:()" + numberPattern + R"() k3:()" + numberPattern + R"() k4:()" + numberPattern + R"() k5:()"
                               + numberPattern + R"() k6:()" + numberPattern + R"() p1:()" + numberPattern + R"() p2:()" + numberPattern + R"())");

    std::regex extrinsicTranslationRegex(R"(translation:()" + numberPattern + R"( )()" + numberPattern + R"( )()" + numberPattern + R"())");

    while(std::getline(file, line)) {
        std::smatch match;

        // Match color intrinsic parameters
        if(std::regex_search(line, match, intrinsicRegex) && line.find("Color Intrinsic") != std::string::npos) {
            color_intr = { std::stof(match[1]), std::stof(match[2]),          std::stof(match[3]),
                           std::stof(match[4]), int16_t(std::stoi(match[5])), int16_t(std::stoi(match[6])) };
        }

        // Match depth intrinsic parameters
        else if(std::regex_search(line, match, intrinsicRegex) && line.find("Depth Intrinsic") != std::string::npos) {
            depth_intr = { std::stof(match[1]), std::stof(match[2]),          std::stof(match[3]),
                           std::stof(match[4]), int16_t(std::stoi(match[5])), int16_t(std::stoi(match[6])) };
        }

        // Match color distortion parameters
        else if(std::regex_search(line, match, distortionRegex) && line.find("Color Distortion") != std::string::npos) {
            color_disto = { std::stof(match[1]), std::stof(match[2]), std::stof(match[3]),
                            std::stof(match[4]), std::stof(match[5]), std::stof(match[6]),
                            std::stof(match[7]), std::stof(match[8]), OBCameraDistortionModel::OB_DISTORTION_BROWN_CONRADY };
        }

        // Match depth distortion parameters
        else if(std::regex_search(line, match, distortionRegex) && line.find("Depth Distortion") != std::string::npos) {
            depth_disto = { std::stof(match[1]), std::stof(match[2]), std::stof(match[3]),
                            std::stof(match[4]), std::stof(match[5]), std::stof(match[6]),
                            std::stof(match[7]), std::stof(match[8]), OBCameraDistortionModel::OB_DISTORTION_BROWN_CONRADY };
        }

        // Match depth to color rotation parameters
        else if(std::regex_search(line, match, extrinsicRotationRegex)) {
            for(int i = 0; i < 9; ++i) {
                trans.rot[i] = std::stof(match[1 + i]);
            }
        }

        // Match depth to color translation parameters
        else if(std::regex_search(line, match, extrinsicTranslationRegex)) {
            for(int i = 0; i < 3; ++i) {
                trans.trans[i] = std::stof(match[1 + i]);
            }
        }
    }

    file.close();
    return 0;
}

int main(int argc, char *argv[]) {
    if(argc < 4) {
        std::cerr << "Usage: %s <param_file> <depth_image_file> <color_image_file>" << std::endl;
        return -1;
    }

    OBCameraIntrinsic  depth_intr, color_intr;
    OBCameraDistortion depth_disto, color_disto;
    OBTransform        transform;
    if(0 != parse_obviewer_data(argv[1], depth_intr, color_intr, depth_disto, color_disto, transform)) {
        std::cerr << "Parse parameter file error." << std::endl;
        return -1;
    }
    libobsensor::AlignImpl *impl = new libobsensor::AlignImpl();
    impl->initialize(depth_intr, depth_disto, color_intr, color_disto, transform, 1, true, true);

    ob_error *err = nullptr;

    FILE *depth_file = fopen(argv[2], "rb");
    if(!depth_file) {
        *err = { ob_status::OB_STATUS_ERROR, "read file error", "fopen", *argv[2], ob_exception_type::OB_EXCEPTION_TYPE_IO };
        std::cerr << err->message << std::endl;
        return -1;
    }

    uint32_t  depth_size = depth_intr.width * depth_intr.height * sizeof(uint16_t);
    uint16_t *depth_data = (uint16_t *)malloc(depth_size);
    fread(depth_data, depth_size, 1, depth_file);
    for(int i = 0; i < depth_intr.width * depth_intr.height; i++) {
        depth_data[i] = _byteswap_ushort(depth_data[i]);
    }
    uint32_t  aligned_depth_size = color_intr.width * color_intr.height * sizeof(uint16_t);
    uint16_t *aligned_depth_data = (uint16_t *)malloc(aligned_depth_size);
    if(0 == impl->D2C(depth_data, depth_intr.width, depth_intr.height, aligned_depth_data, color_intr.width, color_intr.height, nullptr, false)) {
        char nname[256];
        sprintf(nname, "%s_filtered.raw", argv[2]);
        FILE *fp = fopen(nname, "wb");
        if(fp) {
            fwrite(aligned_depth_data, 1, aligned_depth_size, fp);
            fclose(fp);
        }
    }
    free(aligned_depth_data);

    FILE *color_file = fopen(argv[3], "rb");
    if(!color_file) {
        *err = { ob_status::OB_STATUS_ERROR, "read file error", "fopen", *argv[3], ob_exception_type::OB_EXCEPTION_TYPE_IO };
        std::cerr << err->message << std::endl;
        return -1;
    }
    uint32_t color_size = color_intr.width * color_intr.height * sizeof(uint8_t) * 3;
    uint8_t *color_data = (uint8_t *)malloc(color_size);
    fread(color_data, color_size, 1, color_file);
    uint32_t aligned_color_size = depth_intr.width * depth_intr.height * sizeof(uint8_t) * 3;
    uint8_t *aligned_color_data = (uint8_t *)malloc(aligned_color_size);
    if(0 == impl->C2D(depth_data, depth_intr.width, depth_intr.height, color_data, aligned_color_data, color_intr.width, color_intr.height, OB_FORMAT_RGB, false)) {
        char nname[256];
        sprintf(nname, "%s_filtered.raw", argv[3]);
        FILE *fp = fopen(nname, "wb");
        if(fp) {
            fwrite(aligned_color_data, 1, aligned_color_size, fp);
            fclose(fp);
        }
    }

    free(depth_data);
    free(color_data);
    free(aligned_color_data);

    delete impl;
    impl = nullptr;
    return 0;
}
