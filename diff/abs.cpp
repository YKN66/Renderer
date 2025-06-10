// ------------------- abs.cpp -------------------
// 画像(PFM)の各画素についてチャンネル毎に絶対値を取り，新しい PFM として保存します。
// diff.cpp → Gaussian_Blur.cpp に続く 3 段目のユーティリティ。
//
// 使い方:  abs <入力pfm> <出力pfm>
//   例:    abs blurred.pfm abs.pfm
// -------------------------------------------------

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

struct FloatRGB { float r, g, b; };

// readPFM / writePFM は diff.cpp, Gaussian_Blur.cpp と同一実装
// --------------------------------------------------------------
bool readPFM(const std::string& fn, std::vector<FloatRGB>& img, int& w, int& h, float& scale) {
    std::ifstream f(fn, std::ios::binary);
    if (!f) { std::cerr << "open " << fn << " failed\n"; return false; }

    std::string tag;  f >> tag;
    bool isRGB = (tag == "PF");
    if (!isRGB && tag != "Pf") { std::cerr << fn << " not PFM\n"; return false; }

    f >> w >> h >> scale;  f.get();               // eat '\n'
    const int nCh = isRGB ? 3 : 1;
    std::vector<float> buf(w * h * nCh);
    f.read(reinterpret_cast<char*>(buf.data()), buf.size() * sizeof(float));

    img.resize(w * h);
    // bottom‑up → top‑down
    for (int y = 0; y < h; ++y)
      for (int x = 0; x < w; ++x)
      {
        size_t src = (h - 1 - y) * w * nCh + x * nCh;
        FloatRGB& dst = img[y * w + x];
        if (isRGB) {
            dst = { buf[src], buf[src + 1], buf[src + 2] };
        } else {
            dst = { buf[src], buf[src], buf[src] };     // grey→RGB
        }
      }
    return true;
}

bool writePFM(const std::string& fn, const std::vector<FloatRGB>& img, int w, int h, float scale) {
    std::ofstream f(fn, std::ios::binary);
    if (!f) { std::cerr << "write " << fn << " failed\n"; return false; }
    f << "PF\n" << w << ' ' << h << '\n' << scale << '\n';

    for (int y = h - 1; y >= 0; --y)        // top‑down → bottom‑up
      for (int x = 0; x < w; ++x)
      {
        const FloatRGB& p = img[y * w + x];
        f.write(reinterpret_cast<const char*>(&p), sizeof(FloatRGB));
      }
    return true;
}
// --------------------------------------------------------------

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <input.pfm> <output.pfm>\n";
        return 1;
    }
    std::string inF = argv[1];
    std::string outF = argv[2];

    int w, h; float scale; std::vector<FloatRGB> img;
    if (!readPFM(inF, img, w, h, scale)) return 1;

    // 画素毎に絶対値を計算
    for (auto &px : img) {
        px.r = std::fabs(px.r);
        px.g = std::fabs(px.g);
        px.b = std::fabs(px.b);
    }

    if (!writePFM(outF, img, w, h, scale)) return 1;

    std::cout << "abs image written: " << outF << " (" << w << "×" << h << ")\n";
    return 0;
}
