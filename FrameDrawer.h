#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H

#include "Tracking.h"
#include "MapPoint.h"
#include "Map.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include<mutex>


namespace ORB_SLAM2
{

class Tracking;
class Viewer;

class FrameDrawer
{
public:
    //contruye un graficador de frame enlazado con el mapa como puntero
    FrameDrawer(Map* pMap);

    // Update info from the last processed frame.
    void Update(Tracking *pTracker);

    // Draw last processed frame.
    //dibuja el frame , una imagen con informacion y puntos de interes para mostrar
    cv::Mat DrawFrame();

protected:

    //incorpora el texto en la imagen capturada dependiendo del estado del sistema
    void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);

    // Info of the frame to be drawn
    cv::Mat mIm;//imagen capturada , lista para mostrar
    int N;//cantidad de puntos de interes en el frame actual a mostrar
    vector<cv::KeyPoint> mvCurrentKeys; //puntos en el el frame actual
    vector<bool> mvbMap, mvbVO; //puntos que tienen mas/o una observacion en el mapa y en caso contrario en el visual odometry
    bool mbOnlyTracking; //variable para hacer solo el tracking
    int mnTracked, mnTrackedVO;//cantidad de puntos tracked en el mapa y cantidad de puntos en el visual odometry
    vector<cv::KeyPoint> mvIniKeys;//puntos de interes en el frame inicial(referencia)
    vector<int> mvIniMatches; //lista de matches entre los puntos del frame actual y el frame inicial
    int mState;//estado del sistema para incorporar como informacion a la imagen

    Map* mpMap; // enlace al mapa para recuperar informacion

    std::mutex mMutex;
};

} //namespace ORB_SLAM

#endif // FRAMEDRAWER_H
