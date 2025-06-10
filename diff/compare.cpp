// ───────── diff_pipeline.cpp ─────────
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

// ── 共通型 & I/O ──
struct FloatRGB { float r,g,b; };

bool readPFM(const std::string& fn, std::vector<FloatRGB>& img,
             int& w,int& h,float& scale);
bool writePFM(const std::string& fn, const std::vector<FloatRGB>& img,
              int w,int h,float scale);

// ----------------- 1) 差分 -----------------
void makeDiff(const std::vector<FloatRGB>& A,
              const std::vector<FloatRGB>& B,
              std::vector<FloatRGB>& D)
{
    D.resize(A.size());
    for(size_t i=0;i<A.size();++i){
        D[i].r = A[i].r - B[i].r;
        D[i].g = A[i].g - B[i].g;
        D[i].b = A[i].b - B[i].b;
    }
}

// ----------------- 2) ガウシアンぼかし -----------------
void gaussianBlur(std::vector<FloatRGB>& img,int w,int h,float sigma)
{
    int radius = static_cast<int>(std::ceil(2*sigma));
    int sz = 2*radius+1;
    std::vector<float> k(sz);
    float sum=0;
    for(int i=0;i<sz;++i){ int x=i-radius;
        k[i]=std::exp(-(x*x)/(2*sigma*sigma)); sum+=k[i]; }
    for(float& v:k) v/=sum;

    auto blur1D=[&](bool hor){
        std::vector<FloatRGB> out(w*h);
        for(int y=0;y<h;++y)for(int x=0;x<w;++x){
            FloatRGB acc{0,0,0};
            for(int t=-radius;t<=radius;++t){
                int xx = hor? std::clamp(x+t,0,w-1): x;
                int yy = hor? y : std::clamp(y+t,0,h-1);
                const FloatRGB& p = img[yy*w+xx];
                float wgt = k[t+radius];
                acc.r+=p.r*wgt; acc.g+=p.g*wgt; acc.b+=p.b*wgt;
            }
            out[y*w+x]=acc;
        }
        img.swap(out);
    };
    blur1D(true); blur1D(false);
}

// ----------------- 3) 絶対値 -----------------
void absImage(std::vector<FloatRGB>& img){
    for(auto& p:img){
        p.r=std::fabs(p.r); p.g=std::fabs(p.g); p.b=std::fabs(p.b);
    }
}

// ----------------- main -----------------
/*
使い方:
    diff_pipeline <imgA.pfm> <imgB.pfm> <sigma> <out.pfm>
  途中結果 diff/blur が欲しければオプションを付け足す:
    diff_pipeline  A.pfm B.pfm 1.5  final.pfm diff.pfm blur.pfm
*/
int main(int argc,char** argv){
    if(argc!=5 && argc!=7){
        std::cerr<<"Usage: "<<argv[0]
                 <<" <imgA.pfm> <imgB.pfm> <sigma> <outAbs.pfm>"
                 <<" [diffTmp.pfm blurTmp.pfm]\n";
        return 1;
    }
    std::string fileA = argv[1], fileB = argv[2];
    float sigma = std::atof(argv[3]);
    std::string outAbs  = argv[4];
    std::string outDiff = (argc==7)? argv[5] : "";
    std::string outBlur = (argc==7)? argv[6] : "";

    // --- 読込 ---
    int wA,hA; float scA; std::vector<FloatRGB> imgA;
    int wB,hB; float scB; std::vector<FloatRGB> imgB;
    if(!readPFM(fileA,imgA,wA,hA,scA) || !readPFM(fileB,imgB,wB,hB,scB)) return 1;
    if(wA!=wB||hA!=hB){ std::cerr<<"size mismatch\n"; return 1; }

    // --- 1) diff ---
    std::vector<FloatRGB> diff; makeDiff(imgA,imgB,diff);
    if(!outDiff.empty()) writePFM(outDiff,diff,wA,hA,scA);

    // --- 2) blur ---
    gaussianBlur(diff,wA,hA,sigma);
    if(!outBlur.empty()) writePFM(outBlur,diff,wA,hA,scA);

    // --- 3) abs ---
    absImage(diff);
    writePFM(outAbs,diff,wA,hA,scA);

    std::cout<<"pipeline finished: "<<outAbs<<"\n";
    return 0;
}

/* ------- readPFM / writePFM (既存と同じ) ------- */
bool readPFM(const std::string& fn,std::vector<FloatRGB>& img,int& w,int& h,float& scale){
    std::ifstream f(fn,std::ios::binary); if(!f){std::cerr<<"open "<<fn<<" failed\n"; return false;}
    std::string tag; f>>tag; bool isRGB=(tag=="PF"); if(!isRGB&&tag!="Pf"){std::cerr<<"not PFM\n"; return false;}
    f>>w>>h>>scale; f.get();
    int nCh=isRGB?3:1; std::vector<float> buf(w*h*nCh);
    f.read(reinterpret_cast<char*>(buf.data()),buf.size()*4);
    img.resize(w*h);
    for(int y=0;y<h;++y)for(int x=0;x<w;++x){
        size_t s=(h-1-y)*w*nCh+x*nCh;
        FloatRGB& d=img[y*w+x];
        if(isRGB) d={buf[s],buf[s+1],buf[s+2]};
        else      d={buf[s],buf[s],buf[s]};
    }
    return true;
}
bool writePFM(const std::string& fn,const std::vector<FloatRGB>& img,int w,int h,float scale){
    std::ofstream f(fn,std::ios::binary); if(!f){std::cerr<<"write "<<fn<<" failed\n"; return false;}
    f<<"PF\n"<<w<<' '<<h<<'\n'<<scale<<"\n";
    for(int y=h-1;y>=0;--y)for(int x=0;x<w;++x){
        const FloatRGB& p=img[y*w+x];
        f.write(reinterpret_cast<const char*>(&p),sizeof(FloatRGB));
    }
    return true;
}
