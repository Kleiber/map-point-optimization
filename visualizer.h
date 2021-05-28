#ifndef VISUALIZER_H
#define VISUALIZER_H

#include<visualizerframe.h>
#include<visualizermap.h>

class Visualizer{
    public:
        Visualizer(VisualizerFrame* pFrameDrawer, VisualizerMap* pMapDrawer);
        void Run();

        VisualizerFrame* FrameDrawer;
        VisualizerMap* MapDrawer;

        double ViewpointX;
        double ViewpointY;
        double ViewpointZ;
        double ViewpointF;

        double delay;
};

#endif // VISUALIZER_H
