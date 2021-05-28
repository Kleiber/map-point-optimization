#ifndef VISUALIZERFRAME_H
#define VISUALIZERFRAME_H

#include<vector>
#include<string>

#include <opencv2/opencv.hpp>

using namespace std;

class VisualizerFrame{
    public:
        VisualizerFrame(vector< pair< double, pair<string, string> > > &pImageList);

        void DrawInfo(cv::Mat &pImg, cv::Mat &pInfoImg);
        cv::Mat DrawFrame();

        cv::Mat image;

        vector< pair< double, pair<string, string> > > a;
        size_t cont = 0;
};

#endif // VISUALIZERFRAME_H
