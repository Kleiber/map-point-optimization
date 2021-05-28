#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include"Map.h"
#include"MapPoint.h"
#include"KeyFrame.h"
#include<pangolin/pangolin.h>

#include<mutex>

namespace ORB_SLAM2
{

class MapDrawer
{
public:
    //enlazamos el mapa para recuperar informacion y las configuraciones para los dibujos
    MapDrawer(Map* pMap, const string &strSettingPath);

    //puntero del mapa a enlazar
    Map* mpMap;

    void DrawReconstruction(); //dibujar reconstruccion, lista de puntos en el mapa
    void DrawMapPoints();//dibujar los puntos en el mapa(map points)
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph); //dibujar los keyframes en el mapa
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);//dibujar la camara (frame actual)
    void SetCurrentCameraPose(const cv::Mat &Tcw); //asignar la posicion de la cama(frame actual)
    void SetReferenceKeyFrame(KeyFrame *pKF);//asignar el keyframe de referencia
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);//recuperar la matrix opengl de la posicion de la camara para dibujar
    void DrawAxis(); //dibujar los ejes coordenados

private:

    float mKeyFrameSize; //tamano de keyframe que sera dibujado
    float mKeyFrameLineWidth;//tamano de la linea para dibujar el keyframe
    float mGraphLineWidth;//tamano de la linea para dibujar el grafo
    float mPointSize;//tamano del punto para dibujar los map points
    float mCameraSize;//tamno de la camara (frame actual) que sera dibujado
    float mCameraLineWidth;//tamano de la linea para dibujar la camara(frame actual)
    float mAxisLineWidth;//tamaño de las lineas de los ejes coordenados

    cv::Mat mCameraPose; //posicion de la camara(frame actual)

    std::mutex mMutexCamera;
};

} //namespace ORB_SLAM

#endif // MAPDRAWER_H
