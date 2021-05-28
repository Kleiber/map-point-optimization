#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include <vector>
#include <list>
#include <set>

#include "KeyFrame.h"
#include "Frame.h"
#include "ORBVocabulary.h"

#include<mutex>


namespace ORB_SLAM2
{

class KeyFrame;
class Frame;


class KeyFrameDatabase
{
public:

    KeyFrameDatabase(const ORBVocabulary &voc); //inicio una lista de palabra
   void add(KeyFrame* pKF); //adicionar un keyframe a la base de datos utilizamos el descriptor BoW(palabra)
   void erase(KeyFrame* pKF);//buscar la palabra del keyframe y luego buscar el keyframe en la lista para eliminarlo

   void clear();//limpiamos los keyframes que fueron adicionados

   // Loop Detection
   //deteccion de loop del frame en la base de datos , luego con los elementos que comparte mas informacion se busca las similiradidades(palabras en comun)
   //finalmente con los que pasan el filtro se realiza un score con sus vecinos en el grafo de covisibilidad y se seleciona todos aquellos que son mejores
   std::vector<KeyFrame *> DetectLoopCandidates(KeyFrame* pKF, float minScore);

   // Relocalization
   //localizacion del frame en la base de datos , luego con los elementos que comparte mas informacion se busca las similiradidades(palabras en comun)
   //finalmente con los que pasan el filtro se realiza un score con sus vecinos en el grafo de covisibilidad y se seleciona todos aquellos que son mejores
   std::vector<KeyFrame*> DetectRelocalizationCandidates(Frame* F);

protected:

  // Associated vocabulary
  const ORBVocabulary* mpVoc;//vocabulario de palabras

  // Inverted file
  std::vector<list<KeyFrame*> > mvInvertedFile; //lista de keyframes que comparten una palabra

  // Mutex
  std::mutex mMutex;
};

} //namespace ORB_SLAM

#endif
