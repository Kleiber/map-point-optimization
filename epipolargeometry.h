#ifndef EPIPOLARGEOMETRY_H
#define EPIPOLARGEOMETRY_H

#include <opencv2/opencv.hpp>
#include <Thirdparty/DBoW2/DUtils/Random.h>

using namespace std;

class EpipolarGeometry{
    public:
        EpipolarGeometry();

        void Normalize(const vector<cv::KeyPoint> &pKeys, vector<cv::Point2f> &pNormalizePoints, cv::Mat &pT);
        void FindFundamental(vector<bool> &pMatchesInliers, float &pScore, cv::Mat &pF);
        cv::Mat ComputeF(const vector<cv::Point2f> &pP1, const vector<cv::Point2f> &pP2);
        float CheckFundamental(const cv::Mat &pF, vector<bool> &pMatchesInliers, float pSigma);
        void DecomposeE(const cv::Mat &pE, cv::Mat &pR1, cv::Mat &pR2, cv::Mat &pt);
        void Triangulate(const cv::KeyPoint &pkp1, const cv::KeyPoint &pkp2, const cv::Mat &pP1, const cv::Mat &pP2, cv::Mat &pX3D);
        int CheckRT(const cv::Mat &pR, const cv::Mat &pt, const vector<cv::KeyPoint> &pKeys1, const vector<cv::KeyPoint> &pKeys2, const vector< pair<int, int> > &pMatches, vector<bool> &pMatchesInliers, const cv::Mat &pK, vector<cv::Point3f> &p3D, float pth, vector<bool> &pGood, float &pParallax);
        bool ReconstructF(vector<bool> &pMatchesInliers, cv::Mat &pF, cv::Mat &pK, cv::Mat &pR, cv::Mat &pt, vector<cv::Point3f> &pX3D, vector<bool> &pTriangulated, float pminParallax, int pminTriangulated);

     private:

        // Matrix Calibration
        cv::Mat K;

        // Keypoints from Reference Frame (Frame 1)
        vector<cv::KeyPoint> Keys1;

        // Keypoints from Current Frame (Frame 2)
        vector<cv::KeyPoint> Keys2;

        // Current Matches from Reference to Current
        vector< pair<int, int> > Matches;
        vector<bool> Matched;

        // Set of ramdom points (RANSAC)
        vector< vector <size_t> > Random;

        // Maximun iterations (RANSAC)
        int MaxIterations;

        // Standard Deviation and Variance
        float Sigma, Sigma2;

};

#endif // EPIPOLARGEOMETRY_H
