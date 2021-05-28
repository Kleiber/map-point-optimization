#ifndef PLANEFILTERING_H
#define PLANEFILTERING_H

#include <vector>
#include <algorithm>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include "MapPoint.h"

using namespace std;
using namespace cv;

namespace ORB_SLAM2
{


class PlaneFiltering{
    public:

        int w;
        int h;
        float fx;
        float fy;
        float cx;
        float cy;
        float minDepth;
        float maxDepth;
        uint32_t random;

        int maxPoints;
        int numSamples;
        int numLocalSamples;
        int planeSize;
        float WorldPlaneSize;
        float maxError;
        float minInlierFraction;
        float maxDepthDiff;
        int numRetries;
        int sampleRadius;

        PlaneFiltering();
        PlaneFiltering (int _w, int _h, float _fx, float _fy, float _cx, float _cy, float _minDepth, float _maxDepth,
                        int _maxPoints, int _numSamples, int _numLocalSample, int _planeSize, float _WorldPlaneSize,
                        float _maxError, float _minInlierFraction, float _maxDepthDiff, int _numRetries);

        int BSD();
        bool isValidDepth(Point2i &pixel, Mat depthImage);
        Point3f ConvertTo3D(Point2i &pixel, Mat depthImage);
        bool sampleLocation(Mat depthImage, Point2i &pixel, Point3f &point,
                            int rMin, int height, int cMin, int width);
        void filteredPointCloud(Mat depthImage, vector<Point3f>  &pointCloud, vector<Point2i> &pixels,
                                vector<Point3f> &normalCloud, vector<Point3f> &outlierCloud);
        float area(Point2f a, Point2f b, Point2f c);
        vector<Point2f> convexHull(vector<Point2f> points);
};
}
#endif // PLANEFILTERING_H
