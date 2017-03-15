//
// Created by ub1404 on 16. 6. 19.
//

#ifndef GRAPH_VISUALIZER_COLORMAP_H
#define GRAPH_VISUALIZER_COLORMAP_H
struct MyRGB{
    MyRGB(){}
    MyRGB(float r, float g, float b):r(r), g(g), b(b){}
    void print(std::ostream& os){
        os<<"["<<r<<","<<g<<","<<b<<"]";
    }
    bool validate(){
        if(r < 0.0f || r > 1.0f || g < 0.0f || g>1.0f || b<0.0f || b>1.0f)
            return false;
        return true;
    }
    float r,g,b;
};
struct ColorMap{
    ColorMap(const std::vector<MyRGB>& colors){
        this->colors = colors;
        n_seg = colors.size() - 1;
        w = 1.0 / n_seg;
        seg.clear();
        seg.push_back(0.0);
        for(int i=1;i<=n_seg;i++){
            seg.push_back(i*w);
        }
    }

    MyRGB getRGB(const double& pos){
        MyRGB rgb;

        int i = colors.size()-1;
        for(;i>=0;i--){
            if(pos >= seg[i])
                break;
        }
        if(i < 0)
            i=0;
        double ww = (pos - seg[i]) / w;

        rgb.r = colors[i].r + ww*(colors[i+1].r - colors[i].r);
        rgb.g = colors[i].g + ww*(colors[i+1].g - colors[i].g);
        rgb.b = colors[i].b + ww*(colors[i+1].b - colors[i].b);
        return rgb;
    }

    std::vector<MyRGB> colors;
    int n_seg;
    std::vector<double> seg;
    double w;
};


#endif //GRAPH_VISUALIZER_COLORMAP_H
