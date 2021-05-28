#ifndef VISUALIZERMAP_H
#define VISUALIZERMAP_H

#include<vector>
#include<mutex>

#include<pinholecamera.h>

#include<Eigen/Dense>
#include<opencv2/opencv.hpp>
#include<pangolin/pangolin.h>

using namespace std;

class VisualizerMap{
    public:
        VisualizerMap(vector<cv::Mat> &pPosesList, PinholeCamera *pPinhole);

        void DrawKeyFrames();
        void SetCurrentCameraMatrix(cv::Mat &pTcw);
        void GetCurrentCameraMatrix(pangolin::OpenGlMatrix &pTwc);
        void DrawCurrentCameraMatrix(pangolin::OpenGlMatrix &pTwc);

        double width, height;
        double fx, fy, cx, cy;

        double KeyFrameLineWidth;
        double CameraLineWidth;

        cv::Mat CameraPose; //Tcw Transformation to world from camera
        mutex MutexCamera;

        vector<cv::Mat> a;
        size_t cont = 0;
};

#endif // VISUALIZERMAP_H
