#include "hough.h"
#include "peak_detection.h"
#include "voting.h"
#include "octree_t.h"
#include "plane_t.h"
#include "settings.h"
#include <vector>
#include <iostream>
#include <chrono>

#include "Mathematic.h"
#include "accumulatorball_t.h"

inline bool orden (plane_t p1, plane_t p2) { return (p1.representativeness > p2.representativeness);}

accumulatorball_t * kht3d( std::vector<plane_t> &planes, octree_t &father, hough_settings &settings) //const double max_distance, const double distance_discretization, const int phi_cells_length )
{

   // Subdividing Procedure
   father.subdivide(settings);
   
   // Initializes the Accumulator
   accumulatorball_t * accum = new accumulatorball_t(settings.max_point_distance, settings.rho_num, settings.phi_num);

   // Voting Procedure
   std::vector<bin_t> used_bins;
   voting(father, *accum, used_bins, settings.max_point_distance);


   // Peak Detection Procedure
   peak_detection(planes, *accum, used_bins);

   
   for (plane_t &p : planes)
   {
      accum->at(p.ti,p.pi,p.ri).peak = true;
   }

   // Sorting planes by representativeness
   for (unsigned int i = 0; i < planes.size(); i++)
   {
      planes[i].representativeness = 0;
	  for (unsigned int j = 0; j < planes[i].nodes.size(); j++)
         planes[i].representativeness += planes[i].nodes[j]->representativeness;
   }
   std::sort(planes.begin(),planes.end(),orden);


#ifdef debugplanes

   // Coloring planes and points
   for (unsigned int i = 0; i < planes.size(); i++)
   {
      Vector4d cor;
      switch(i%6)
      {
      case 0:cor = Vector4d((int)(255/(int)(i/6+1)),0,0)/255.0;break;
      case 1:cor = Vector4d(0,(int)(255/(int)(i/6+1)),0)/255.0;break;
      case 2:cor = Vector4d(0,0,(int)(255/(int)(i/6+1)))/255.0;break;
      case 3:cor = Vector4d(0,(int)(255/(int)(i/6+1)),(int)(255/(int)(i/6+1)))/255.0;break;
      case 4:cor = Vector4d((int)(255/(int)(i/6+1)),0,(int)(255/(int)(i/6+1)))/255.0;break;
      case 5:cor = Vector4d((int)(255/(int)(i/6+1)),(int)(255/(int)(i/6+1)),0)/255.0;break;
      }

      planes[i].m_color = cor;

	  for (unsigned int j = 0; j < planes[i].nodes.size(); j++)
         planes[i].nodes[j]->color = cor;
   }
   
   
   for (unsigned int i = 0; i < father.m_points.size(); i++)
   {
	   for (unsigned int p = 0; p < planes.size(); p++)
      {
         if (planes[p].distance2plane(father.m_points[i]) < settings.max_distance2plane)
         {
            father.m_colors[i] = planes[p].m_color;
            break;
         }
      }
   }
#endif

   used_bins.clear();

   return accum;
}
