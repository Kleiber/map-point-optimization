#pragma once

#include "Mathematic.h"
#include "Vector4d.h"
#include "settings.h"
#include <Eigen/Dense>
#include <GL/glut.h>
#include <omp.h>


class octree_t {
public:

   octree_t();

   void subdivide(hough_settings &settings);
   void least_variance_direction(void);
   void remove_outliers(void);
   void print_points(void);
   void show(bool type, int height);
   void show(bool type);
   void clear(void);
   void get_nodes(std::vector< octree_t*> &nodes);
   double distance2plane(Vector4d &point);
   Eigen::Matrix3d fast_covariance_matrix(void);
   Eigen::Matrix3d m_covariance;
   std::vector<Vector4d> m_points, m_colors;
   std::vector<int> m_indexes;

   octree_t * m_children, * m_root;
   Vector4d normal1, normal2, normal3, m_middle, m_centroid, color;
   double variance1, variance2, variance3, m_size, representativeness;
   short m_level;
   bool coplanar;
   int votes;
};


