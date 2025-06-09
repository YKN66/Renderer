#pragma once
#include <fstream>
#include <vector>
#include <string>

inline void write_pfm(const std::string& filename, const std::vector<float>& data, int width, int height) {

    std::ofstream ofs(filename, std::ios::binary);
    if (!ofs) throw std::runtime_error("cannot open " + filename);
    ofs << "PF\n" << width << ' ' << height << "\n-1.0\n";

    ofs.write(reinterpret_cast<const char*>(data.data()), sizeof(float) * data.size());
}
