// ------------------- Gaussian_Blur.cpp -------------------
#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <algorithm>
#include <string>

struct FloatRGB{ float r,g,b; };
bool readPFM(const std::string&, std::vector<FloatRGB>&, int&, int&, float&);
bool writePFM(const std::string&, const std::vector<FloatRGB>&, int,int,float);

// ── separable Gaussian blur ──
void gaussianBlur(std::vector<FloatRGB>& img, int w,int h, float sigma) {
    int radius = static_cast<int>(std::ceil(2*sigma));
    int sz = 2*radius+1;

    std::vector<float> k(sz);
    float sum=0;
    for(int i=0;i<sz;++i){
        int x=i-radius;
        k[i]=std::exp(-(x*x)/(2*sigma*sigma));
        sum+=k[i];
    }
    for(float& v:k) v/=sum;

    auto blur1D=[&](bool horizontal){
        std::vector<FloatRGB> out(w*h);
        for(int y=0;y<h;++y) for(int x=0;x<w;++x){
            FloatRGB acc{0,0,0};
            for(int t=-radius;t<=radius;++t){
                int xx = horizontal? std::clamp(x+t,0,w-1): x;
                int yy = horizontal? y : std::clamp(y+t,0,h-1);
                const FloatRGB& p = img[yy*w+xx];
                float wgt = k[t+radius];
                acc.r+=p.r*wgt; acc.g+=p.g*wgt; acc.b+=p.b*wgt;
            }
            out[y*w+x]=acc;
        }
        img.swap(out);
    };
    blur1D(true);   // horizontal
    blur1D(false);  // vertical
}

int main(int argc,char** argv) {
    if(argc<4){
        std::cerr<<"Usage: "<<argv[0]<<" diff.pfm blurred.pfm sigma\n"; return 1;
    }
    std::string inF=argv[1], outF=argv[2]; float sigma=std::atof(argv[3]);
    int w,h; float scale; std::vector<FloatRGB> img;
    if(!readPFM(inF,img,w,h,scale)) return 1;

    gaussianBlur(img,w,h,sigma);

    if(!writePFM(outF,img,w,h,scale)) return 1;
    std::cout<<"blurred written: "<<outF<<"\n";
    int size_blue = sigma*4 + 1;
    std::cout << "blurred size: " << size_blue << std::endl;
}

/* ------------------------------------------------------- */
/* -- 以下 readPFM / writePFM は diff.cpp と同じ実装 ----- */
bool readPFM(const std::string& fn, std::vector<FloatRGB>& img, int& w, int& h, float& scale) {
    std::ifstream f(fn,std::ios::binary); if(!f){std::cerr<<"open "<<fn<<" failed\n";return false;}
    std::string tag; f>>tag; bool isRGB=(tag=="PF"); if(!isRGB&&tag!="Pf"){std::cerr<<"not PFM\n";return false;}
    f>>w>>h>>scale; f.get();
    int nCh=isRGB?3:1; std::vector<float> buf(w*h*nCh);
    f.read(reinterpret_cast<char*>(buf.data()),buf.size()*4);
    img.resize(w*h);
    for(int y=0;y<h;++y)for(int x=0;x<w;++x){
        size_t src=(h-1-y)*w*nCh+x*nCh;
        FloatRGB& dst=img[y*w+x];
        if(isRGB) dst={buf[src],buf[src+1],buf[src+2]};
        else      dst={buf[src],buf[src],buf[src]};
    }
    return true;
}
bool writePFM(const std::string& fn,const std::vector<FloatRGB>& img, int w,int h,float scale) {
    std::ofstream f(fn,std::ios::binary); if(!f){std::cerr<<"write "<<fn<<" failed\n"; return false;}
    f<<"PF\n"<<w<<' '<<h<<'\n'<<scale<<"\n";
    for(int y=h-1;y>=0;--y)for(int x=0;x<w;++x){
        const FloatRGB& p=img[y*w+x];
        f.write(reinterpret_cast<const char*>(&p),sizeof(FloatRGB));
    }
    return true;
}
