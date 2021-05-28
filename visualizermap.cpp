#include "visualizermap.h"

VisualizerMap::VisualizerMap(vector<cv::Mat> &pPosesList, PinholeCamera *pPinhole){

    fx = pPinhole->fx;
    fy = pPinhole->fy;
    cx = pPinhole->cx;
    cy = pPinhole->cy;
    width = pPinhole->width;
    height = pPinhole->height;

    KeyFrameLineWidth = 2.0;
    CameraLineWidth = 3.0;

    a = pPosesList;
}

void VisualizerMap :: SetCurrentCameraMatrix(cv::Mat &pTcw){
    unique_lock<mutex> lock(MutexCamera);
    CameraPose = pTcw.clone();
}

void VisualizerMap :: GetCurrentCameraMatrix(pangolin::OpenGlMatrix &pTwc){

    if(!CameraPose.empty()){
        cv::Mat R(3, 3, CV_32F);
        cv::Mat t(3, 1, CV_32F);

        {
            unique_lock<mutex> lock(MutexCamera);
            R = CameraPose.rowRange(0,3).colRange(0,3).t();
            t = -R * CameraPose.rowRange(0,3).col(3);
        }

        pTwc.m[0] = R.at<float>(0,0);
        pTwc.m[1] = R.at<float>(1,0);
        pTwc.m[2] = R.at<float>(2,0);
        pTwc.m[3]  = 0.0;

        pTwc.m[4] = R.at<float>(0,1);
        pTwc.m[5] = R.at<float>(1,1);
        pTwc.m[6] = R.at<float>(2,1);
        pTwc.m[7]  = 0.0;

        pTwc.m[8] = R.at<float>(0,2);
        pTwc.m[9] = R.at<float>(1,2);
        pTwc.m[10] = R.at<float>(2,2);
        pTwc.m[11] = 0.0;

        pTwc.m[12] = t.at<float>(0);
        pTwc.m[13] = t.at<float>(1);
        pTwc.m[14] = t.at<float>(2);
        pTwc.m[15] = 1.0;

    }else{
        pTwc.SetIdentity();
    }
}

void VisualizerMap :: DrawCurrentCameraMatrix(pangolin::OpenGlMatrix &pTwc){

    glPushMatrix();
        glMultMatrixd(pTwc.m);
        glLineWidth(CameraLineWidth);
        glColor3f(0.0f,1.0f,0.0f);
        glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(0.05*(0-cx)/fx,0.05*(0-cy)/fy,0.05);
            glVertex3f(0,0,0);
            glVertex3f(0.05*(0-cx)/fx,0.05*(height-1-cy)/fy,0.05);
            glVertex3f(0,0,0);
            glVertex3f(0.05*(width-1-cx)/fx,0.05*(height-1-cy)/fy,0.05);
            glVertex3f(0,0,0);
            glVertex3f(0.05*(width-1-cx)/fx,0.05*(0-cy)/fy,0.05);

            glVertex3f(0.05*(width-1-cx)/fx,0.05*(0-cy)/fy,0.05);
            glVertex3f(0.05*(width-1-cx)/fx,0.05*(height-1-cy)/fy,0.05);

            glVertex3f(0.05*(width-1-cx)/fx,0.05*(height-1-cy)/fy,0.05);
            glVertex3f(0.05*(0-cx)/fx,0.05*(height-1-cy)/fy,0.05);

            glVertex3f(0.05*(0-cx)/fx,0.05*(height-1-cy)/fy,0.05);
            glVertex3f(0.05*(0-cx)/fx,0.05*(0-cy)/fy,0.05);

            glVertex3f(0.05*(0-cx)/fx,0.05*(0-cy)/fy,0.05);
            glVertex3f(0.05*(width-1-cx)/fx,0.05*(0-cy)/fy,0.05);
        glEnd();
    glPopMatrix();
}

void VisualizerMap ::DrawKeyFrames(){

    for(size_t i = cont; i < cont  + 1; i++){
        cv::Mat Twc = a[i];
        glPushMatrix();
            glMultMatrixf(Twc.ptr<GLfloat>(0));
            glLineWidth(KeyFrameLineWidth);
            glColor3f(0.0f,0.0f,1.0f);
            glBegin(GL_LINES);
                glVertex3f(0,0,0);
                glVertex3f(0.05*(0-cx)/fx,0.05*(0-cy)/fy,0.05);
                glVertex3f(0,0,0);
                glVertex3f(0.05*(0-cx)/fx,0.05*(height-1-cy)/fy,0.05);
                glVertex3f(0,0,0);
                glVertex3f(0.05*(width-1-cx)/fx,0.05*(height-1-cy)/fy,0.05);
                glVertex3f(0,0,0);
                glVertex3f(0.05*(width-1-cx)/fx,0.05*(0-cy)/fy,0.05);

                glVertex3f(0.05*(width-1-cx)/fx,0.05*(0-cy)/fy,0.05);
                glVertex3f(0.05*(width-1-cx)/fx,0.05*(height-1-cy)/fy,0.05);

                glVertex3f(0.05*(width-1-cx)/fx,0.05*(height-1-cy)/fy,0.05);
                glVertex3f(0.05*(0-cx)/fx,0.05*(height-1-cy)/fy,0.05);

                glVertex3f(0.05*(0-cx)/fx,0.05*(height-1-cy)/fy,0.05);
                glVertex3f(0.05*(0-cx)/fx,0.05*(0-cy)/fy,0.05);

                glVertex3f(0.05*(0-cx)/fx,0.05*(0-cy)/fy,0.05);
                glVertex3f(0.05*(width-1-cx)/fx,0.05*(0-cy)/fy,0.05);
            glEnd();
        glPopMatrix();
    }
}
