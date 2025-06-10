// ------------------- diff.cpp -------------------
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>

// ───────────── 共有ヘルパ ─────────────
struct FloatRGB { float r, g, b; };

bool readPFM(const std::string& fn, std::vector<FloatRGB>& img, int& w, int& h, float& scale) {
    std::ifstream f(fn, std::ios::binary);
    if (!f) { std::cerr << "open " << fn << " failed\n"; return false; }

    std::string tag;  f >> tag;
    bool isRGB = (tag == "PF");
    if (!isRGB && tag != "Pf") { std::cerr << fn << " not PFM\n"; return false; }

    f >> w >> h >> scale;  f.get();               // eat '\n'
    const int nCh = isRGB ? 3 : 1;
    std::vector<float> buf(w * h * nCh);
    f.read(reinterpret_cast<char*>(buf.data()), buf.size()*sizeof(float));

    img.resize(w * h);
    // bottom‑up → top‑down
    for (int y = 0; y < h; ++y)
      for (int x = 0; x < w; ++x)
      {
        size_t src = (h-1-y)*w*nCh + x*nCh;
        FloatRGB& dst = img[y*w + x];
        if (isRGB) { dst = {buf[src], buf[src+1], buf[src+2]}; }
        else       { dst = {buf[src], buf[src], buf[src]}; }     // grey→RGB
      }
    return true;
}

bool writePFM(const std::string& fn, const std::vector<FloatRGB>& img, int w, int h, float scale=-1.f) {
    std::ofstream f(fn, std::ios::binary);
    if (!f) { std::cerr << "write " << fn << " failed\n"; return false; }
    f << "PF\n" << w << ' ' << h << '\n' << scale << '\n';

    for (int y = h-1; y >= 0; --y)        // top‑down → bottom‑up
      for (int x = 0; x < w; ++x)
      {
        const FloatRGB& p = img[y*w + x];
        f.write(reinterpret_cast<const char*>(&p), sizeof(FloatRGB));
      }
    return true;
}

// ───────────── main ─────────────
int main(int argc, char** argv) {
    if (argc < 3 || argc > 4) {
        std::cerr << "Usage: " << argv[0] << " <imgA.pfm> <imgB.pfm> [out.pfm]\n";
        return 1;
    }
    std::string fileA = argv[1];
    std::string fileB = argv[2];
    std::string fileOut = (argc == 4) ? argv[3] : "diff.pfm";

    int wA, hA; float scaleA; std::vector<FloatRGB> imgA;
    int wB, hB; float scaleB; std::vector<FloatRGB> imgB;

    if (!readPFM(fileA, imgA, wA, hA, scaleA)) return 1;
    if (!readPFM(fileB, imgB, wB, hB, scaleB)) return 1;

    if (wA != wB || hA != hB) {
        std::cerr << "image size mismatch: " << wA << "x" << hA << " vs " << wB << "x" << hB << "\n";
        return 1;
    }
    if (std::fabs(scaleA - scaleB) > 1e-6f) {
        std::cerr << "warning: scale factors differ (" << scaleA << " vs " << scaleB << ") – using first one.\n";
    }

    std::vector<FloatRGB> diff(imgA.size());
    for (size_t i = 0; i < diff.size(); ++i) {
        diff[i].r = imgA[i].r - imgB[i].r;
        diff[i].g = imgA[i].g - imgB[i].g;
        diff[i].b = imgA[i].b - imgB[i].b;
    }

    if (!writePFM(fileOut, diff, wA, hA, scaleA)) return 1;

    std::cout << "diff image written: " << fileOut << " (" << wA << "×" << hA << ")\n";
    return 0;
}
