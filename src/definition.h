#ifndef DEFINITION_H
#define DEFINITION_H

#include <vector>
#include <Eigen/Core>

namespace uav{

  //definition
  typedef Eigen::Vector3d Point;
  typedef Eigen::Vector4d Point4d;
  typedef std::vector<Point> Polytope;
  typedef Point (*trajectory)(Point, double);
  typedef std::vector<Point4d> Polytope4d;

  const int INFI = INT_MAX;
  //const int INFI = 5; //for visualization

}

#endif
