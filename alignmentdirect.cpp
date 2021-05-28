#include "alignmentdirect.h"

AlignmentDirect::AlignmentDirect(int pwidth, int pheight, cv::Mat pK){

    width   = pwidth;
    height  = pheight;

    K = pK;

    gx.reserve(width*height*sizeof(float));
    gy.reserve(width*height*sizeof(float));
    x.reserve(width*height*sizeof(float));
    y.reserve(width*height*sizeof(float));
    z.reserve(width*height*sizeof(float));
    residual.reserve(width*height*sizeof(float));
    weight.reserve(width*height*sizeof(float));

    idepthVar.reserve(width*height*sizeof(float));
    d.reserve(width*height*sizeof(float));

    sizeResidual = 0;

    iterationNumber = 0;

    diverged = false;
}

cv::Mat AlignmentDirect :: skew_SO3(cv::Mat &pw){
    /*
     * w = [wx wy wz]'
     *
     *     |  0  -wz   wy|
     * w = | wz    0  -wx|
     *     |-wy   wx    0|
     *
     */

    double vwx = pw.at<double>(0,0);
    double vwy = pw.at<double>(1,0);
    double vwz = pw.at<double>(2,0);

    cv::Mat vw  = cv::Mat(3, 3, CV_64F);
    vw.at<double>(0,0) = 0;
    vw.at<double>(0,1) =-vwz;
    vw.at<double>(0,2) = vwy;
    vw.at<double>(1,0) = vwz;
    vw.at<double>(1,1) = 0;
    vw.at<double>(1,2) =-vwx;
    vw.at<double>(2,0) =-vwy;
    vw.at<double>(2,1) = vwx;
    vw.at<double>(2,2) = 0;

    return vw;
}

void AlignmentDirect :: log_SO3(cv::Mat &pR, cv::Mat &pw){

    /*
     *
     *     |r00 r01 r02|
     * R = |r10 r11 r12|
     *     |r20 r21 r22|
     *
     * log(R) = 0.5 * (theta/sin(theta)) * (R - R')
     * cos(theta)  = 0.5 * (trace(R) - 1)   &  cos(theta) within [-1 , 1]
     *
     *     |  0  -wz   wy|
     * w = | wz    0  -wx|
     *     |-wy   wx    0|
     *
     * w = [wx wy wz]'
     *
     */

    cv::Mat vw;

    double vvalue = 0.5 * (cv::trace(pR)[0] - 1);

    if(fabs(vvalue) > 0.9999 && fabs(vvalue) < 1.00001) vw = 0.5 * (pR - pR.t());
    else vw = (0.5 * acos(vvalue) / (sqrt(1 - vvalue * vvalue))) * (pR - pR.t());

    pw.at<double>(0,0) = vw.at<double>(2,1);
    pw.at<double>(1,0) = vw.at<double>(0,2);
    pw.at<double>(2,0) = vw.at<double>(1,0);
}

void AlignmentDirect :: exp_SO3(cv::Mat &pR, cv::Mat &pw){

    /*
     *                   | 0  -wz  wy|
     * w = [wx wy wz]' = | wz  0  -wx|
     *                   |-wy  wx  0 |
     *
     * theta = sqrt(wx*wx + wy*wy + wz*wz)
     *
     * R = exp(w) = I + (sin(theta)/theta)*(w) + ((1 - cos(theta))/theta*theta)*(w*w)
     *
     *     |r00 r01 r02|
     * R = |r10 r11 r12|
     *     |r20 r21 r22|
     *
     */

    double vwx = pw.at<double>(0,0);
    double vwy = pw.at<double>(1,0);
    double vwz = pw.at<double>(2,0);

    cv::Mat veye = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat vw  = skew_SO3(pw);

    double vtheta = sqrt(vwx * vwx + vwy * vwy + vwz * vwz);

    if(vtheta < 0.00015) pR = veye + vw + (0.5 * vw * vw);
    else pR = veye + (sin(vtheta)/vtheta) * vw + (1 - cos(vtheta))/(vtheta*vtheta) * (vw * vw);
}


void AlignmentDirect :: log_SE3(cv::Mat &pR, cv::Mat &pt, cv::Mat &pw, cv::Mat &pv){

    /*
     *      |r00 r01 r02|       |t0|
     * R =  |r10 r11 r12|   t = |t1|
     *      |r20 r21 r22|       |t2|
     *
     *
     *     |R t|   |log(R) inv(A)*t|
     * log |0 1| = | 0         0   |
     *
     * cos(theta)  = 0.5 * (trace(R) - 1)   &  cos(theta) within [-1 , 1]
     *
     * log(R) = 0.5 * (theta/sin(theta)) * (R - R')
     *
     *     |  0  -wz   wy|
     * w = | wz    0  -wx|
     *     |-wy   wx    0|
     *
     * w = [wx wy wz]'
     *
     * A = I + ((1 - cos(theta))/(theta*theta))*(w) + ((theta - sin(theta))/(theta*theta*theta))*(w*w)
     *
     * v = inv(A) * t;
     *
     * w = [wx wy wz]'
     * v = [vx  vy vz]'
     */

    log_SO3(pR, pw);

    double vwx = pw.at<double>(0,0);
    double vwy = pw.at<double>(1,0);
    double vwz = pw.at<double>(2,0);

    cv::Mat vA;
    cv::Mat veye = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat vw   = skew_SO3(pw);

    double vtheta = sqrt(vwx * vwx + vwy * vwy + vwz * vwz);

    if(vtheta < 0.000015) vA = veye + 0.5 *(vw) + 0,166666667*(vw * vw);
    else vA = veye + ((1 - cos(vtheta))/(vtheta*vtheta))*(vw) + ((vtheta - sin(vtheta))/(vtheta*vtheta*vtheta))*(vw * vw);

    pv = vA.inv() * pt;

}

void AlignmentDirect :: exp_SE3(cv::Mat &pR, cv::Mat &pt, cv::Mat &pw, cv::Mat &pv){

    /*
     *     |w v|   |exp(w) Av|
     * exp |0 0| = |0       1|
     *
     *
     * v = [vx vy vz]
     *
     *                  | 0  -wz  wy|
     * w = [wx wy wz] = | wz  0  -wx|
     *                  |-wy  wx  0 |
     *
     * theta = sqrt(wx*wx + wy*wy + wz*wz)
     *
     * R = Exp(w) = I + (sin(theta)/theta)*(w) + ((1 - cos(theta))/theta*theta)*(w*w)
     *
     * A = I + ((1 - cos(theta))/(theta*theta))*(w) + ((theta - sin(theta))/(theta*theta*theta))*(w*w)
     *
     * t = A * v;
     *
     *     |r00 r01 r02|      |tx|
     * R = |r10 r11 r12|   t =|ty|
     *     |r20 r21 r22|      |tz|
     *
     */

    exp_SO3(pR, pw);

    double vwx = pw.at<double>(0,0);
    double vwy = pw.at<double>(1,0);
    double vwz = pw.at<double>(2,0);

    cv::Mat vA;
    cv::Mat veye = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat vw   = skew_SO3(pw);

    double vtheta = sqrt(vwx * vwx + vwy * vwy + vwz * vwz);

    if(vtheta < 0.000015) vA = veye + 0.5 *(vw) + 0,166666667*(vw * vw);
    else vA = veye + ((1 - cos(vtheta))/(vtheta*vtheta))*(vw) + ((vtheta - sin(vtheta))/(vtheta*vtheta*vtheta))*(vw * vw);

    pt = vA * pv;
}

void AlignmentDirect :: calculateGradient(vector<cv::Mat> &pgradient, cv::Mat &pimage, int pwidth, int pheight){

    /*
     *   -------
     *   |a|b|c|
     *   -------                     |  dx  |                         |  dx  |  |f - d|
     *   |d|e|f|    gradiente(x,y) = |  dy  |      gradiente((1,1)) = |  dy  |= |h - b|
     *   -------                     |I(x,y)|                         |I(1,1)|  |  e  |
     *   |g|h|i|
     *   -------
     */

    for(int i = 1; i < pheight - 1; i++){
        for(int j = 1; j < pwidth - 1; j++){
            cv::Mat vaux(3, 1, CV_64F);
            vaux.at<float>(0,0) = pimage.at<float>(i, j + 1) - pimage.at<float>(i, j - 1);
            vaux.at<float>(1,0) = pimage.at<float>(i + 1, j) - pimage.at<float>(i - 1, i);
            vaux.at<float>(2,0) = pimage.at<float>(i, j);

            int vpos = pwidth * i + j;
            pgradient[vpos] = vaux;
        }
    }
}


float AlignmentDirect :: calculateResidual(vector<cv::Mat> &ppoint, vector<float> &pintensity, vector<float> &pvariance,
                                          vector<cv::Mat> &pgradient, cv::Mat &pK, cv::Mat pR, cv::Mat &pt, int pwidth, int pheight){

    float vfx = pK.at<float>(0,0);
    float vfy = pK.at<float>(1,1);
    float vcx = pK.at<float>(0,2);
    float vcy = pK.at<float>(1,2);

    sizeResidual = 0;
    float vsumResUnweighted = 0;
    int vgoodCount = 0;
    float sxx = 0, syy = 0, sx = 0,sy = 0,sw = 0;

    for(size_t i = 0; i < ppoint.size(); i++){
        cv::Mat vpoint = pR * ppoint[i] + pt;

        float u = (vpoint.at<float>(0,0)/vpoint.at<float>(2,0)) * vfx + vcx;
        float v = (vpoint.at<float>(1,0)/vpoint.at<float>(2,0)) * vfy + vcy;

        if((u > 1 && u < pwidth - 2) && (v > 1 && v < pheight - 2)){

            int ui = (int) u;
            int vi = (int) v;
            float vdu = u - ui;
            float vdv = v - vi;

            int vpos = pwidth * ui + vi;

            cv::Mat vinterpolation = (1 - vdu - vdv + vdu*vdv) * pgradient[vpos] +
                                     (vdu - vdu*vdv) * pgradient[vpos + 1] +
                                     (vdv - vdu*vdv) * pgradient[vpos + width] +
                                     (vdu*vdv) * pgradient[vpos + pwidth + 1];

            float vc1 = affineEstimation_a * pintensity[i] + affineEstimation_b;
            float vc2 = vinterpolation.at<float>(2,0);
            float vres = vc1 - vc2;
            float vweight = 0;

            if(fabs(vres) < 5.0f) vweight = 1;
            else vweight = 5.0f/fabs(vres);

            sxx += vc1 * vc1 * vweight;
            syy += vc2 * vc2 * vweight;
            sx += vc1 * vweight;
            sy += vc2 * vweight;
            sw += vweight;

            bool isGood = vres * vres / (MAX_DIFF_CONSTANT + MAX_DIFF_GRAD_MULT*(vinterpolation.at<float>(0,0)*vinterpolation.at<float>(0,0) + vinterpolation.at<float>(1,0)*vinterpolation.at<float>(1,0))) < 1;

            x[sizeResidual] = vpoint.at<float>(0,0);
            y[sizeResidual] = vpoint.at<float>(1,0);
            z[sizeResidual] = vpoint.at<float>(2,0);
            gx[sizeResidual] = vfx * vinterpolation.at<float>(0,0);
            gy[sizeResidual] = vfy * vinterpolation.at<float>(1,0);
            residual[sizeResidual] = vres;
            d[sizeResidual] = 1.0 / ppoint[i].at<float>(2,0);
            idepthVar[sizeResidual] = pvariance[i];

            sizeResidual++;

            if(isGood){
                vsumResUnweighted += vres * vres;
                vgoodCount++;
            }
        }
    }

    affineEstimation_a_lastIt = sqrtf((syy - sy*sy/sw) / (sxx - sx*sx/sw));
    affineEstimation_b_lastIt = (sy - affineEstimation_a_lastIt*sx)/sw;

    return vsumResUnweighted/vgoodCount;
}

float AlignmentDirect :: calculateWeight(cv::Mat &pt){

    float vsumRes = 0;
    float vhuber = 3;
    float vcameraPixelNoise2 = 4*4;
    float vvar_weight = 1.0;

    for(int i = 0; i < sizeResidual; i++){

        float s = vvar_weight * idepthVar[i];

        // calc dw/dd (first 2 components):
        float vg0 = (pt.at<float>(0,0) * z[i] - pt.at<float>(0,2) * x[i]) / (z[i]*z[i]*d[i]);
        float vg1 = (pt.at<float>(1,0) * z[i] - pt.at<float>(0,2) * y[i]) / (z[i]*z[i]*d[i]);

        // calc w_p
        float vdrpdd = gx[i] * vg0 + gy[i] * vg1;
        float vw_p = 1.0f / ((vcameraPixelNoise2) + s * vdrpdd * vdrpdd);
        float vweighted_rp = fabs(residual[i] * sqrtf(vw_p));
        float vwh = fabs(vweighted_rp < (vhuber/2) ? 1 : (vhuber/2) / vweighted_rp);
        vsumRes += vwh * vw_p * residual[i] * residual[i];

        weight[i] = vwh * vw_p;
    }

    return vsumRes / sizeResidual;
}

void AlignmentDirect :: calculateUpdate(cv::Mat &pA, cv::Mat &pb, float &perror){

    /*
     * Residual function
     *
     * r(x) = I1(x) - I2(x)
     *
     * Rotation (angular velocity) and traslation (linear velocity) from frame2 to frame1
     *
     *      | 1  -wz  wy|      |vx|
     * R =  | wz  1  -wx|  t = |vy|
     *      |-wy  wx  1 |      |vz|
     *
     * Gradient , ptical flow on the frame2
     *
     *     |gx|      |dx|      |x1|      |x2|
     * g = |gy|   d =|dy|  p1 =|y1|  p2 =|y2|
     *                         |z1|      |z2|
     *
     * Jacobian of the residual function
     *
     * p2 = R * p1 + t
     *
     * |x2|   | x1 - y1*wz + z1*wy + vx|
     * |y2| = | x1*wz + y1 - z1*wx + vy|
     * |z2|   |-x1*wy + y1*wx + z1 + vz|
     *
     * r(x) = g . d = gx * dx + gy * dy = gx * (x2/z2 - x1/z1) + gy * (y2/z2 - y1/z1)
     *
     * J = |der(r(x),vx) der(r(x),vy) der(r(x),vz) der(r(x),wx) der(r(x),wy) der(r(x),wz)|'
     *
     * A =  J * J' * w(x)
     * b = -J * r(x) * w(x)
     * E = |vx vy vz wx wy wz|'
     *
     * error = r(x) * r(x) * w(x)
     *
     * A* E = b (solve this system and find E)
     *
     */

    pA = cv::Mat::zeros(6, 6, CV_64F);
    pb = cv::Mat::zeros(6, 1, CV_64F);
    perror = 0;

    for(int i = 0; i < sizeResidual; i++){

        cv::Mat vJ(6, 1, CV_64F);
        vJ.at<float>(0,0) = (1.0/z[i]) * gx[i];
        vJ.at<float>(1,0) = (1.0/z[i]) * gy[i];
        vJ.at<float>(2,0) = (-x[i] * (1.0/(z[i]*z[i]))) * gx[i] + (-y[i] * (1.0/(z[i]*z[i]))) * gy[i];
        vJ.at<float>(3,0) = (-x[i] * y[i] * (1.0/(z[i]*z[i]))) * gx[i] + (-(1.0 + y[i] * y[i] * (1.0/(z[i]*z[i])))) * gy[i];
        vJ.at<float>(4,0) = (1.0 + x[i] * x[i] * (1.0/(z[i]*z[i]))) * gx[i] + (x[i] * y[i] * (1.0/(z[i]*z[i]))) * gy[i];
        vJ.at<float>(5,0) = (-y[i] * (1.0/z[i])) * gx[i] + (x[i] + (1.0/z[i])) * gy[i];

        pA += vJ * vJ.t() * weight[i];
        pb -= vJ * residual[i] * weight[i];
        perror += residual[i] * residual[i] * weight[i];
    }

    pA /= sizeResidual;
    pb /= sizeResidual;
    perror /= sizeResidual;
}

