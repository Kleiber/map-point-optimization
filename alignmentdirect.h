#ifndef ALIGNMENTDIRECT_H
#define ALIGNMENTDIRECT_H

#include <opencv2/opencv.hpp>
#include <Eigen/Core>

#define MAX_DIFF_CONSTANT  (40.0f*40.0f)
#define MAX_DIFF_GRAD_MULT (0.5f*0.5f)

using namespace std;

class AlignmentDirect{
    public:

        AlignmentDirect(int pwidth, int pheight, cv::Mat K);

        cv::Mat skew_SO3(cv::Mat &pw);
        void exp_SO3(cv::Mat &pR, cv::Mat &pw);
        void log_SO3(cv::Mat &pR, cv::Mat &pw);
        void log_SE3(cv::Mat &pR, cv::Mat &pt, cv::Mat &pw, cv::Mat &pv);
        void exp_SE3(cv::Mat &pR, cv::Mat &pt, cv::Mat &pw, cv::Mat &pv);

        void calculateGradient(vector<cv::Mat> &pgradient, cv::Mat &pimage, int pwidth, int pheight);
        void calculateVariance(vector<float> &pvariance, cv::Mat &pimage, int pwidth, int pheight);

        float calculateWeight(cv::Mat &pt);
        float calculateResidual(vector<cv::Mat> &ppoint, vector<float> &pintensity, vector<float> &pvariance, vector<cv::Mat> &pgradient, cv::Mat &pK, cv::Mat pR, cv::Mat &pt, int pwidth, int pheight);
        void  calculateUpdate(cv::Mat &pA, cv::Mat &pb, float &perror);

        void track();

        int width;
        int height;
        cv::Mat K;

        vector<float> residual;
        vector<float> gx;
        vector<float> gy;
        vector<float> x;
        vector<float> y;
        vector<float> z;
        vector<float> d;
        vector<float> idepthVar;
        vector<float> weight;

        int sizeResidual;

        float pointUsage;
        float lastGoodCount;
        float lastBadCount;
        float lastResidual;

        int iterationNumber;

        bool diverged;

        float affineEstimation_a;
        float affineEstimation_b;
        float affineEstimation_a_lastIt;
        float affineEstimation_b_lastIt;

};

#endif // ALIGNMENTDIRECT_H
