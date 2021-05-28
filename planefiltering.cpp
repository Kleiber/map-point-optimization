#include "planefiltering.h"

namespace ORB_SLAM2
{
PlaneFiltering :: PlaneFiltering(){

}

PlaneFiltering :: PlaneFiltering (int _w, int _h, float _fx, float _fy, float _cx, float _cy, float _minDepth, float _maxDepth,
                                  int _maxPoints, int _numSamples, int _numLocalSample, int _planeSize, float _WorldPlaneSize,
                                  float _maxError, float _minInlierFraction, float _maxDepthDiff, int _numRetries){
    w =_w;
    h = _h;
    fx = _fx;
    fy = _fy;
    cx = _cx;
    cy = _cy;

    minDepth = _minDepth;
    maxDepth = _maxDepth;

    maxPoints = _maxPoints;
    numSamples = _numSamples;
    numLocalSamples = _numLocalSample;
    planeSize = _planeSize;
    WorldPlaneSize = _WorldPlaneSize;
    maxError = _maxError;
    minInlierFraction = _minInlierFraction;
    maxDepthDiff = _maxDepthDiff;
    numRetries = _numRetries;

    random = 0;
}

int PlaneFiltering :: BSD(){

    random = 1103515245 * random + 12345;
    return random;
}

bool PlaneFiltering :: isValidDepth(Point2i &pixel, Mat depthImage){

    float depth = depthImage.at<float>(pixel.y, pixel.x);
    return depth >= minDepth && depth <= maxDepth;
}

Point3f PlaneFiltering :: ConvertTo3D(Point2i &pixel, Mat depthImage){

    float depth = depthImage.at<float>(pixel.y, pixel.x);

    float z = depth/5000.0;
    float x = -(pixel.x - cx) * z / fx;
    float y = -(pixel.y - cy) * z / fy;

    return Point3f(x, y, z);
}

bool PlaneFiltering :: sampleLocation(Mat depthImage, Point2i &pixel, Point3f &point,
                                      int rMin, int height, int cMin, int width){

    int iterations = 0;
    bool valid  = 0;
    while(!valid && iterations <= numRetries){
        pixel.y = rMin + abs(BSD() % height);
        pixel.x = cMin + abs(BSD() % width);
        valid = isValidDepth(pixel, depthImage);
        iterations++;
    }

    if(valid){
        point = ConvertTo3D(pixel, depthImage);
    }
    return valid;
}

void PlaneFiltering :: filteredPointCloud(Mat depthImage, vector<Point3f>  &pointCloud,
                                       vector<Point2i> &pixels, vector<Point3f> &normalCloud,
                                       vector<Point3f> &outlierCloud ){

    pointCloud.clear();
    pixels.clear();
    normalCloud.clear();
    outlierCloud.clear();

    pointCloud.reserve(2 * maxPoints);
    pixels.reserve(2 * maxPoints);
    normalCloud.reserve(2 * maxPoints);
    outlierCloud.reserve(numSamples);

    float minInlier = minInlierFraction * numLocalSamples;
    float maxOutliers = (1.0 - minInlierFraction) * numLocalSamples;
    float planeSizeH = planeSize / 2.0;
    float w2 = w - planeSize;
    float h2 = h - planeSize;

    int numPlanes = 0;
    int numPoints = 0;

    Point2i pixel1, pixel2, pixel3, pixel;
    Point3f point1, point2, point3, point;
    vector<Point2i> neighborhoodPixels;
    vector<Point3f> neighborhoodInliers;

    float distance , meanDeapth;
    int rMin, rMax, cMin, cMax, dR, dC, xcentre, ycentre;

    for(int i = 0; i < numSamples && numPoints < maxPoints; i++){

        neighborhoodInliers.clear();
        neighborhoodPixels.clear();

        sampleLocation(depthImage, pixel1, point1, planeSizeH, h2, planeSizeH, w2);
        sampleLocation(depthImage, pixel2, point2, pixel1.y - planeSizeH, planeSize, pixel1.x - planeSizeH, planeSize);
        sampleLocation(depthImage, pixel3, point3, pixel1.y - planeSizeH, planeSize, pixel1.x - planeSizeH, planeSize);

        Point3f normal = (point1 - point2).cross(point3 - point2);
        normal = normal/sqrt(normal.dot(normal));
        distance = point1.dot(normal);
        meanDeapth = (point1.z + point2.z + point3.z) / 3.0;
        xcentre = ceil((pixel1.x + pixel2.x + pixel3.x) / 3.0);
        ycentre = ceil((pixel1.y + pixel2.y + pixel3.y) / 3.0);

        rMin = max(0, ycentre - planeSize);
        rMax = min(h - 1, ycentre + planeSize);
        cMin = max(0, xcentre - planeSize);
        cMax = min(w - 1, xcentre + planeSize);
        dR = rMax - rMin;
        dC = cMax - cMin;

        int inliers = 0;
        int outliers = 0;

        for(int j = 0; j < numLocalSamples && outliers < maxOutliers; j++){
            sampleLocation(depthImage, pixel, point, rMin, dR, cMin, dC);
            float error = fabs(normal.dot(point) - distance);

            if(error < maxError && point.z < meanDeapth + maxDepthDiff && point.z > meanDeapth - maxDepthDiff){
                inliers++;
                neighborhoodInliers.push_back(point);
                neighborhoodPixels.push_back(pixel);
            }else outliers++;
        }

        if(inliers <= minInlier && inliers > 3){

            for(size_t j = 0; j < neighborhoodInliers.size(); j++){
                pointCloud.push_back(neighborhoodInliers[j]);
                normalCloud.push_back(normal);
                pixels.push_back(neighborhoodPixels[j]);
            }

            pointCloud.push_back(point1);
            pointCloud.push_back(point2);
            pointCloud.push_back(point3);
            normalCloud.push_back(normal);
            normalCloud.push_back(normal);
            normalCloud.push_back(normal);
            pixels.push_back(pixel1);
            pixels.push_back(pixel2);
            pixels.push_back(pixel3);

            numPoints += neighborhoodInliers.size() + 3;
            numPlanes ++;
        }else{
            for(size_t j = 0; j < neighborhoodInliers.size(); j++)
                outlierCloud.push_back(neighborhoodInliers[i]);

            outlierCloud.push_back(point1);
            outlierCloud.push_back(point2);
            outlierCloud.push_back(point3);
        }
    }
}

float PlaneFiltering :: area(Point2f a, Point2f b, Point2f c){
    return (b - a).cross(c - a);
}

vector<Point2f> PlaneFiltering ::convexHull(vector<Point2f> points){
    int n = points.size();
    int k = 0;
    Point2f H[1000000];

    sort(points.begin(),points.end(), [](const Point2f &a, const Point2f &b){ if(a.x != b.x) return a.x < b.x; return a.y < b.y; });

    for(int i = 0;i < n; i++){
        while(k >= 2 && area(H[k - 2], H[k - 1], points[i]) >= 0) k--;
        H[k++] = points[i];
    }

    int t = k + 1;

    for(int i = n - 2; i >= 0; i--){
        while(k >= t && area(H[k - 2], H[k - 1], points[i]) >= 0) k--;
        H[k++] = points[i];
    }

    return vector <Point2f> (H, H + k - 1);
}

}

