#include "stb_image_write.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <stdexcept>

bool read_pfm(const std::string& filename, std::vector<float>& data, int& width, int& height) {
    std::ifstream ifs(filename, std::ios::binary);
    if (!ifs) {
        std::cerr << "Error: Cannot open file " << filename << std::endl;
        return false;
    }

    std::string header;
    ifs >> header;
    if (header != "PF") {
        std::cerr << "Error: Not a PFM file." << std::endl;
        return false;
    }

    ifs >> width >> height;

    float scale;
    ifs >> scale;
    ifs.ignore(1);

    int total = width * height * 3;
    data.resize(total);

    ifs.read(reinterpret_cast<char*>(data.data()), sizeof(float) * total);

    std::vector<float> temp = data;
    for (int y = 0; y < height; ++y) {
        std::copy(
            temp.begin() + (height - 1 - y) * width * 3,
            temp.begin() + (height - y) * width * 3,
            data.begin() + y * width * 3
        );
    }

    return true;
}

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: pfm_to_png input.pfm output.png" << std::endl;
        return 1;
    }

    std::string pfm_file = argv[1];
    std::string png_file = argv[2];

    std::vector<float> pfm_data;
    int width, height;

    if (!read_pfm(pfm_file, pfm_data, width, height)) {
        return 1;
    }

    std::vector<unsigned char> png_data(width * height * 3);
    for (int i = 0; i < width * height * 3; ++i) {
        float val = std::max(0.0f, std::min(1.0f, pfm_data[i])); // clamp
        png_data[i] = static_cast<unsigned char>(255.99f * std::pow(val, 1.0f / 2.2f)); // gamma補正
    }

    if (!stbi_write_png(png_file.c_str(), width, height, 3, png_data.data(), width * 3)) {
        std::cerr << "Error: Failed to write PNG file." << std::endl;
        return 1;
    }

    std::cout << "Converted " << pfm_file << " to " << png_file << std::endl;
    return 0;
}
