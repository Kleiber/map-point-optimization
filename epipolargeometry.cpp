#include "epipolargeometry.h"

EpipolarGeometry::EpipolarGeometry(){
}

void EpipolarGeometry :: Normalize(const vector<cv::KeyPoint> &pKeys, vector<cv::Point2f> &pNormalizePoints, cv::Mat &pT){

    //-- Implementation of Revisiting Hartley’s Normalized Eight-Point Algorithm
    /*
     *      |s1  0  -s1*m1|
     *  T = |0   s2 -s2*m2|
     *      |0   0     1  |
     */

    const int vN = pKeys.size();
    pNormalizePoints.resize(vN);

    float vmeanX = 0;
    float vmeanY = 0;
    for(int i = 0; i < vN; i++){
        vmeanX += pKeys[i].pt.x;
        vmeanY += pKeys[i].pt.y;
    }
    vmeanX = vmeanX/vN;
    vmeanY = vmeanY/vN;

    float vmeanDevX = 0;
    float vmeanDevY = 0;
    for(int i = 0; i < vN; i++){
        pNormalizePoints[i].x = pKeys[i].pt.x - vmeanX;
        pNormalizePoints[i].y = pKeys[i].pt.y - vmeanY;
        vmeanDevX += fabs(pNormalizePoints[i].x);
        vmeanDevY += fabs(pNormalizePoints[i].y);
    }
    vmeanDevX = vmeanDevX/vN;
    vmeanDevY = vmeanDevY/vN;

    float vsX = 1.0/vmeanDevX;
    float vsY = 1.0/vmeanDevY;
    for(int i = 0; i < vN; i++){
        pNormalizePoints[i].x = pNormalizePoints[i].x * vsX;
        pNormalizePoints[i].y = pNormalizePoints[i].y * vsY;
    }

    pT = cv::Mat::eye(3, 3, CV_32F);
    pT.at<float>(0,0) = vsX;
    pT.at<float>(1,1) = vsY;
    pT.at<float>(0,2) = -vmeanX * vsX;
    pT.at<float>(1,2) = -vmeanY * vsY;

}

cv::Mat EpipolarGeometry :: ComputeF(const vector<cv::Point2f> &pP1, const vector<cv::Point2f> &pP2){

    // -- Implementation of 8-point Algorithm
    /*
     *              x2t * F * x1 = 0
     *
     *        |f11 f12 f13|
     *   F =  |f21 f22 f23|    ^    rank(F) = 2
     *        |f31 f32 f33|
     */

    const int vN = pP1.size();
    cv::Mat vA(vN, 9, CV_32F);

    for(int i = 0; i < vN; i++){
        const float u1 = pP1[i].x;
        const float v1 = pP1[i].y;
        const float u2 = pP2[i].x;
        const float v2 = pP2[i].y;

        vA.at<float>(i,0) = u2 * u1;
        vA.at<float>(i,1) = u2 * v1;
        vA.at<float>(i,2) = u2;
        vA.at<float>(i,3) = v2 * u1;
        vA.at<float>(i,4) = v2 * v1;
        vA.at<float>(i,5) = v2;
        vA.at<float>(i,6) = u1;
        vA.at<float>(i,7) = v1;
        vA.at<float>(i,8) = 1;
    }

    cv::Mat vU, vW, vVt;
    cv::SVDecomp(vA, vW, vU, vVt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    cv::Mat vFpre = vVt.row(8).reshape(0,3);
    cv::SVDecomp(vFpre, vW, vU, vVt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    vW.at<float>(2) = 0;

    return vU * cv::Mat::diag(vW) * vVt;
}

float EpipolarGeometry :: CheckFundamental(const cv::Mat &pF, vector<bool> &pMatchesInliers, float pSigma){

    //-- Implementation of Robust Detection of Degenerate Configurations while Estimating the Fundamental Matrix
    /*
     *  d^2 = 3.841 * (1.0/(sigma*sigma))^2 (threshold)
     *
     *  l2 = F x1  = (a2, b2, c2) (Reprojection error in second image)
     *  l1 = x2t F = (a1, b1, c1) (Reprojection error in first image)
     *
     */

    const int vN = Matches.size();

    const float f11 = pF.at<float>(0,0);
    const float f12 = pF.at<float>(0,1);
    const float f13 = pF.at<float>(0,2);
    const float f21 = pF.at<float>(1,0);
    const float f22 = pF.at<float>(1,1);
    const float f23 = pF.at<float>(1,2);
    const float f31 = pF.at<float>(2,0);
    const float f32 = pF.at<float>(2,1);
    const float f33 = pF.at<float>(2,2);

    pMatchesInliers.resize(vN);

    float vscore = 0;
    const float vth = 3.841;
    const float vthScore = 5.991;
    const float vinvSigmaSquare = 1.0/(pSigma * pSigma);

    for(int i = 0; i < vN; i++){

        bool vIn = true;

        const cv::KeyPoint &vkp1 = Keys1[Matches[i].first ];
        const cv::KeyPoint &vkp2 = Keys2[Matches[i].second];
        const float u1 = vkp1.pt.x;
        const float v1 = vkp1.pt.y;
        const float u2 = vkp2.pt.x;
        const float v2 = vkp2.pt.y;


        const float va2 = f11 * u1 + f12 * v1 + f13;
        const float vb2 = f21 * u1 + f22 * v1 + f23;
        const float vc2 = f31 * u1 + f32 * v1 + f33;
        const float vnum2 = va2 * u2 + vb2 * v2 + vc2;
        const float vsquareDist1 = (vnum2 * vnum2)/(va2 * va2 + vb2 * vb2);
        const float vchiSquare1 = vsquareDist1 * vinvSigmaSquare;

        if(vchiSquare1 > vth) vIn = false;
        else vscore += vthScore - vchiSquare1;


        const float va1 = f11 * u2 + f21 * v2 + f31;
        const float vb1 = f12 * u2 + f22 * v2 + f32;
        const float vc1 = f13 * u2 + f23 * v2 + f33;
        const float vnum1 = va1 * u1 + vb1 * v1 + vc1;
        const float vsquareDist2 = (vnum1 * vnum1)/(va1 * va1 + vb1 * vb1);
        const float vchiSquare2 = vsquareDist2 * vinvSigmaSquare;

        if(vchiSquare2 > vth) vIn = false;
        else vscore += vthScore - vchiSquare2;


        if(vIn) pMatchesInliers[i] = true;
        else pMatchesInliers[i] = false;
    }

    return vscore;
}

void EpipolarGeometry :: FindFundamental(vector<bool> &pMatchesInliers, float &pScore, cv::Mat &pF){

    // -- Implementation of Revisiting Hartley’s Normalized Eight-Point Algorithm
    /*
     *   F = T2' * F * T1
     */

    vector<cv::Point2f> vPn1, vPn2;
    cv::Mat vT1, vT2;
    Normalize(Keys1, vPn1, vT1);
    Normalize(Keys2, vPn2, vT2);
    cv::Mat vT2t = vT2.t();

    pScore = 0.0;

    vector<cv::Point2f> vPn1i(8);
    vector<cv::Point2f> vPn2i(8);
    vector<bool> vcurrentInliers;
    cv::Mat vFi;
    float vcurrentScore;

    for(int i = 0; i < MaxIterations; i++){

        for(int j = 0; j < 8; j++){
            int vind = Random[i][j];
            vPn1i[j] = vPn1[Matches[vind].first ];
            vPn2i[j] = vPn2[Matches[vind].second];
        }

        cv::Mat vFn = ComputeF(vPn1i, vPn2i);
        vFi = vT2t * vFn * vT1;
        vcurrentScore = CheckFundamental(vFi, vcurrentInliers, Sigma);

        if(vcurrentScore > pScore){
            pF = vFi.clone();
            pMatchesInliers = vcurrentInliers;
            pScore = vcurrentScore;
        }
    }
}

void EpipolarGeometry :: DecomposeE(const cv::Mat &pE, cv::Mat &pR1, cv::Mat &pR2, cv::Mat &pt){

    /*
     * E = U D V'
     *
     * U = [u1 u2 t]
     *
     *        |0 -1 0|                   |0 -1 0|'
     * R =  U |1  0 0| V'   (ou)   R = U |1  0 0| V'
     *        |0  0 0|                   |0  0 0|
     */

    cv::Mat vU, vW, vVt;
    cv::SVD::compute(pE, vW, vU, vVt);

    vU.col(2).copyTo(pt);
    pt = pt/cv::norm(pt);

    cv::Mat vD(3, 3, CV_32F, cv::Scalar(0));
    vD.at<float>(0,1) = -1;
    vD.at<float>(1,0) =  1;
    vD.at<float>(2,2) =  1;

    pR1 = vU * vD * vVt;
    if(cv::determinant(pR1) < 0) pR1 = -pR1;

    pR2 = vU * vD.t() * vVt;
    if(cv::determinant(pR2) < 0) pR2 = -pR2;
}

void EpipolarGeometry ::Triangulate(const cv::KeyPoint &pkp1, const cv::KeyPoint &pkp2, const cv::Mat &pP1, const cv::Mat &pP2, cv::Mat &pX3D){

    //-- Linear Triangulation
    /*
     *                 |0   -az   ay| |bx|
     * (a) cross (b) = |az    0  -ax| |by|
     *                 |-ay  ax    0| |bz|
     *
     *     |p11 p12 p13 p14|   |p1'|
     * P = |p21 p22 p23 p24| = |p2'|
     *     |p31 p32 p33 p34|   |p3'|
     *
     * sx = PX
     *
     * (x) cross (PX) = 0
     *
     * u * p3' * X - p1' * X     = 0
     * v * p3' * X - p2' * X     = 0
     * u * p2' * X - v * p1' * X = 0
     *
     * |u*p3' - p1'|
     * |v*p3' - p2'| X = 0
     *
     * AX = 0 (and) rank(A) = n-1  (then) X = Vn
     *
     */

    cv::Mat vA(4, 4, CV_32F);

    vA.row(0) = pkp1.pt.x * pP1.row(2) - pP1.row(0);
    vA.row(1) = pkp1.pt.y * pP1.row(2) - pP1.row(1);
    vA.row(2) = pkp2.pt.x * pP2.row(2) - pP2.row(0);
    vA.row(3) = pkp2.pt.y * pP2.row(2) - pP2.row(1);

    cv::Mat vU, vW, vVt;
    cv::SVD::compute(vA, vW, vU, vVt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    pX3D = vVt.row(3).t();
    pX3D = pX3D.rowRange(0,3)/pX3D.at<float>(3);
}

int EpipolarGeometry :: CheckRT(const cv::Mat &pR, const cv::Mat &pt, const vector<cv::KeyPoint> &pKeys1, const vector<cv::KeyPoint> &pKeys2, const vector< pair<int, int> > &pMatches, vector<bool> &pMatchesInliers, const cv::Mat &pK, vector<cv::Point3f> &pX3D, float pth, vector<bool> &pGood, float &pParallax){

    const float fx = pK.at<float>(0,0);
    const float fy = pK.at<float>(1,1);
    const float cx = pK.at<float>(0,2);
    const float cy = pK.at<float>(1,2);

    pGood = vector<bool>(pKeys1.size(), false);
    pX3D.resize(pKeys1.size());

    vector<float> vCosParallax;
    vCosParallax.reserve(pKeys1.size());

    // Camera 1 Projection Matrix K[I|0]
    cv::Mat vP1(3, 4, CV_32F, cv::Scalar(0));
    K.copyTo(vP1.rowRange(0, 3).colRange(0, 3));
    cv::Mat vO1 = cv::Mat::zeros(3, 1, CV_32F);

    // Camera 2 Projection Matrix K[R|t]
    cv::Mat vP2(3, 4, CV_32F);
    pR.copyTo(vP2.rowRange(0, 3).colRange(0, 3));
    pt.copyTo(vP2.rowRange(0, 3).col(3));
    vP2 = pK * vP2;
    cv::Mat vO2 = -pR.t() * pt;

    int vGood = 0;

    for(size_t i = 0; i < pMatches.size(); i++){

        if(!pMatchesInliers[i]) continue;

        const cv::KeyPoint &vkp1 = pKeys1[pMatches[i].first ];
        const cv::KeyPoint &vkp2 = pKeys2[pMatches[i].second];
        cv:: Mat vX3D1;

        Triangulate(vkp1, vkp2, vP1, vP2, vX3D1);

        if(!isfinite(vX3D1.at<float>(0)) || !isfinite(vX3D1.at<float>(1)) || !isfinite(vX3D1.at<float>(2))){
            pGood[pMatches[i].first] = false;
            continue;
        }

        cv::Mat vnormal1 = vX3D1 - vO1;
        cv::Mat vnormal2 = vX3D1 - vO2;
        float vdist1 = cv::norm(vnormal1);
        float vdist2 = cv::norm(vnormal2);
        float vcosParallax = (vnormal1.dot(vnormal2))/(vdist1 * vdist2);

        // Check depth in front of first camera (only if enough parallax, as "infinite" points can easily go to negative depth)
        if(vX3D1.at<float>(2) <= 0 && vcosParallax < 0.99998) continue;

        cv::Mat vX3D2 = pR * vX3D1 + pt;

        // Check depth in front of second camera (only if enough parallax, as "infinite" points can easily go to negative depth)
        if(vX3D2.at<float>(2) <= 0 && vcosParallax < 0.99998) continue;

        // Check reprojection error in first image
        float vinvz1 = 1.0/vX3D1.at<float>(2);
        float vx1 = fx * vX3D1.at<float>(0) * vinvz1 + cx;
        float vy1 = fy * vX3D1.at<float>(1) * vinvz1 + cy;
        float vsquareError1 = (vx1 - vkp1.pt.x) * (vx1 - vkp1.pt.x) + (vy1 - vkp1.pt.y) * (vy1 - vkp1.pt.y);
        if(vsquareError1 > pth) continue;

        // Check reprojection error in second image
        float vinvz2 = 1.0/vX3D2.at<float>(2);
        float vx2 = fx * vX3D2.at<float>(0) * vinvz2 + cx;
        float vy2 = fy * vX3D2.at<float>(1) * vinvz2 + cy;
        float vsquareError2 = (vx2 - vkp2.pt.x) * (vx2 - vkp2.pt.x) + (vy2 - vkp2.pt.y) * (vy2 - vkp2.pt.y);
        if(vsquareError2 > pth) continue;

        vCosParallax.push_back(vcosParallax);
        pX3D[pMatches[i].first] = cv::Point3f(vX3D1.at<float>(0), vX3D1.at<float>(1), vX3D1.at<float>(2));
        vGood++;

        if(vcosParallax < 0.99998) pGood[pMatches[i].first] = true;
    }

    if(vGood > 0){
        sort(vCosParallax.begin(), vCosParallax.end());
        int vind = min(50, int(vCosParallax.size() - 1));
        pParallax = acos(vCosParallax[vind]) * 180/CV_PI;
    }else{
        pParallax = 0;
    }

    return vGood;
}

bool EpipolarGeometry :: ReconstructF(vector<bool> &pMatchesInliers, cv::Mat &pF, cv::Mat &pK, cv::Mat &pR, cv::Mat &pt, vector<cv::Point3f> &pX3D, vector<bool> &pTriangulated, float pminParallax, int pminTriangulated){

    /*
     *  E = K' F K = (t) cross (R)
     *
     *  P1 = K [I | 0]
     *  P2 = K [R | t]
     *
     *  hypotheses
     *
     *  P2 = [ U  D  V' |  t ]
     *  P2 = [ U  D' V' |  t ]
     *  P2 = [ U  D  V' | -t ]
     *  P2 = [ U  D' V' | -t ]
     *
     */

    cv::Mat vE = pK.t() * pF * pK;
    cv::Mat vR1, vR2, vt;
    DecomposeE(vE, vR1, vR2, vt);
    cv::Mat vt1 =  vt;
    cv::Mat vt2 = -vt;

    vector<cv::Point3f> vX3D1, vX3D2, vX3D3, vX3D4;
    vector<bool> vTriangulated1, vTriangulated2, vTriangulated3, vTriangulated4;
    float vparallax1, vparallax2, vparallax3, vparallax4;
    int vGood1, vGood2, vGood3, vGood4;

    vGood1 = CheckRT(vR1, vt1, Keys1, Keys2, Matches, pMatchesInliers, pK, vX3D1, 4.0 * Sigma2, vTriangulated1, vparallax1);
    vGood2 = CheckRT(vR2, vt1, Keys1, Keys2, Matches, pMatchesInliers, pK, vX3D2, 4.0 * Sigma2, vTriangulated2, vparallax2);
    vGood3 = CheckRT(vR1, vt2, Keys1, Keys2, Matches, pMatchesInliers, pK, vX3D3, 4.0 * Sigma2, vTriangulated3, vparallax3);
    vGood4 = CheckRT(vR2, vt2, Keys1, Keys2, Matches, pMatchesInliers, pK, vX3D4, 4.0 * Sigma2, vTriangulated4, vparallax4);

    int vmaxGood = max(vGood1, max(vGood2, max(vGood3, vGood4)));

    pR = cv::Mat();
    pt = cv::Mat();

    int vN = 0;
    for(size_t i = 0; i < pMatchesInliers.size(); i++){
        if(pMatchesInliers[i]) vN++;
    }

    int vminGood = max(static_cast<int>(0.9 * vN), pminTriangulated);
    int vsimilar = 0;

    if(vGood1 > 0.7 * vmaxGood) vsimilar++;
    if(vGood2 > 0.7 * vmaxGood) vsimilar++;
    if(vGood3 > 0.7 * vmaxGood) vsimilar++;
    if(vGood4 > 0.7 * vmaxGood) vsimilar++;

    // If there is not a clear winner or not enough triangulated points reject initialization
    if(vmaxGood < vminGood || vsimilar > 1) return false;

    // If best reconstruction has enough parallax initialize
    if(vmaxGood == vGood1){
        if(vparallax1 > pminParallax){
            pX3D = vX3D1;
            pTriangulated = vTriangulated1;
            vR1.copyTo(pR);
            vt1.copyTo(pt);
            return true;
        }
    }else{
        if(vmaxGood == vGood2){
            if(vparallax2 > pminParallax){
                pX3D = vX3D2;
                pTriangulated = vTriangulated2;
                vR2.copyTo(pR);
                vt1.copyTo(pt);
                return true;
            }
        }else{
            if(vmaxGood == vGood3){
                if(vparallax3 > pminParallax){
                    pX3D = vX3D3;
                    pTriangulated = vTriangulated3;
                    vR1.copyTo(pR);
                    vt2.copyTo(pt);
                    return true;
                }
            }else{
                if(vmaxGood == vGood4){
                    if(vparallax4 > pminParallax){
                        pX3D = vX3D4;
                        pTriangulated = vTriangulated4;
                        vR2.copyTo(pR);
                        vt2.copyTo(pt);
                        return true;
                    }
                }
            }
        }
    }

    return false;
}
