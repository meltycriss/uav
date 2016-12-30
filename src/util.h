#ifndef UTIL_H
#define UTIL_H

#include <vector>
#include <Eigen/Core>
#include "iris/iris.h"
#include "definition.h"

namespace uav{

  Polytope minkowskiSum(const Polytope &lhs, const Polytope &rhs);

  //return polytope after moving
  Polytope polytopeAfterMove(const Polytope &poly, double time, trajectory m);

  //return poly4d appending x to each vertice in poly3d
  Polytope4d poly3dToPoly4d(const Polytope &poly3d, double x);

  //return bounds generated from lb to ub
  iris::Polyhedron getPolyFromBounds(const Point4d&, const Point4d&);

  //distance between two Polytope is the distance between their centroid
  double getMinInterDis(const std::vector<Polytope> &uavs);

  //return centroid of a polytope
  Point getCentroid(const Polytope &uav);

  //return wrapper radius of a polytope
  double getRadius(const Polytope &uav);

  Eigen::MatrixXd quatRotateVecDiff(const Eigen::Vector4d &q, const Eigen::Vector3d &v);

  Eigen::MatrixXd quatProductDiff(const Eigen::Vector4d &lhs, const Eigen::Vector4d &rhs);

  Eigen::MatrixXd quatConjugateDiff(const Eigen::Vector4d &q);

  Eigen::MatrixXd dcross(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2, Eigen::MatrixXd &dv1, Eigen::MatrixXd &dv2);

  //IRIS result visualization helper
  void reducePolyDim(Eigen::MatrixXd &A, Eigen::VectorXd &b, int dim);

  //IRIS result visualization helper
  //plug A[*][dim] = val to slice
  void reducePolyDim(Eigen::MatrixXd &A, Eigen::VectorXd &b, int dim, double val);

  void removeRow(Eigen::MatrixXd& matrix, unsigned int rowToRemove);

  void removeRow(Eigen::VectorXd& vec, unsigned int rowToRemove);

  void removeColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove);

  //The hungarian method for assignment method
  //return rows belong to cols
  std::vector<int> hungarian(const Eigen::MatrixXd &matCost);

  //return index of min element in a vector
  int minOfVec(const Eigen::VectorXd &vec);

  //return vec element wise add x
  Eigen::VectorXd vecAdd(const Eigen::VectorXd &vec, double x);

  //absolute coordinates to relative coordinates respect to centroid
  std::vector<Polytope> absToRela(const std::vector<Polytope> &abs, const Point &centroid);

  //relative coordinates to absolute coordinates respect to centroid
  std::vector<Polytope> relaToAbs(const std::vector<Polytope> &rela, const Point &centroid);

  //relative coordinates to absolute coordinates respect to centroid
  std::vector<Point> relaToAbs(const std::vector<Point> &rela, const Point &centroid);

  //move polys with tsq param
  std::vector<Polytope> tsqTransPolyVec(const std::vector<Polytope> &polys, const Vector8d &tsq);

  //move a point with tsq param
  Point tsqTransPoint(const Point &p, const Vector8d &tsq);

  //return centroid distance matrix
  Eigen::MatrixXd getDisMat(const std::vector<Polytope> &lhs, const std::vector<Polytope> &rhs);
  
  //return centroid distance matrix
  Eigen::MatrixXd getDisMat(const std::vector<Polytope> &lhs, const std::vector<Point> &rhs);

  //move uavs centered with centroid to uavs centered with goal
  Polytope uavMoveTo(const Point &goal, const Polytope &uav);

}

#endif
