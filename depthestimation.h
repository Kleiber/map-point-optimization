#ifndef DEPTHESTIMATION_H
#define DEPTHESTIMATION_H

#include <list>
#include <vector>
#include <set>

#include <Eigen/Dense>
#include <KeyFrame.h>
#include <MapPoint.h>

#include <se3.h>
#include <seed.h>
#include <pinholecamera.h>

#define MAX_LENGTH_EPIPOLAR_LINE 50.0
#define STEP_EPIPOLAR_LINE 0.7

class DepthEstimation{

    public:
        DepthEstimation(PinholeCamera* pPinhole);

        bool MeanDepth(std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > &pImagePoint, double &pdepthMean, double &pdepthMin);
        bool EpipolarMatchNCC(SE3 &pT_curr_ref, Seed &pSeed, Eigen::Vector3d &pbestPixel);
        Eigen::Vector3d Triangulation(SE3 &pT_ref_curr, Eigen::Vector3d &pbeam_ref, Eigen::Vector3d &pbeam_curr);
        double TriangulationUncertainty(SE3 &pT_ref_curr, Eigen::Vector3d &pbeam_ref, double pdepth);

        void initKeyFrameSeeds(ORB_SLAM2::KeyFrame* pFrame_ref);
        void updateKeyFrameSeeds(ORB_SLAM2::KeyFrame* pFrame_curr);

        std::list<Seed, Eigen::aligned_allocator<Seed> > SeedList;
        PinholeCamera* Pinhole;

        int conta_Seed, total_Seed;
};

#endif // DEPTHESTIMATION_H
