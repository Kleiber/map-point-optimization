#include "visualizer.h"
#include<iostream>

#include<pangolin/pangolin.h>

Visualizer :: Visualizer(VisualizerFrame *pFrameDrawer, VisualizerMap *pMapDrawer){

    FrameDrawer = pFrameDrawer;
    MapDrawer = pMapDrawer;

    ViewpointX =  0.0;
    ViewpointY = -0.7;
    ViewpointZ = -1.8;
    ViewpointF =  500;

    double fps = 30.0;;
    delay = 1e3/fps;
}

void Visualizer :: Run(){
    pangolin::CreateWindowAndBind("Vizualizer",1024,768);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState RenderCam(pangolin::ProjectionMatrix(1024, 768, ViewpointF, ViewpointF, 512, 389, 0.1, 1000),
                                          pangolin::ModelViewLookAt(ViewpointX, ViewpointY, ViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));

    pangolin::View &Cam = pangolin::CreateDisplay().SetBounds(0.0, 1.0, pangolin::Attach::Pix(0), 1.0, -1024.0f/768.0f)
                                                   .SetHandler(new pangolin::Handler3D(RenderCam));
    cv::namedWindow("Current Frame");

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    while(1){
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        MapDrawer->GetCurrentCameraMatrix(Twc);
        RenderCam.Follow(Twc);

        Cam.Activate(RenderCam);
        glClearColor(0.0f,0.0f,0.0f,0.0f);
        MapDrawer->DrawKeyFrames();
        pangolin::FinishFrame();

        cv::Mat img = FrameDrawer->DrawFrame();
        cv::imshow("Current Frame",img);
        cv::waitKey(delay);
        FrameDrawer->cont++;
        MapDrawer->cont++;
    }
}

