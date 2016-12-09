#include <iostream>
#include "util.h"
#include <cmath>
#include "drake/math/quaternion.h"
using namespace std;

namespace uav{

  Polytope minkowskiSum(const Polytope &lhs, const Polytope &rhs){
    Polytope res;
    for(int i=0; i<lhs.size(); ++i){
      for(int j=0; j<rhs.size(); ++j){
        Point p1 = lhs[i], p2 = rhs[j];
        res.push_back(p1+p2);
      }
    }
    return res;
  }
  // return polytope after moving
  Polytope polytopeAfterMove(const Polytope &poly, double time, trajectory m){
    Polytope res;
    for(int i=0; i<poly.size(); ++i){
      Point p = poly[i];
      Point pAfterMove = (*m)(p, time);
      res.push_back(pAfterMove);
    }
    return res;
  }

  // return poly4d appending x to each vertice in poly3d
  Polytope4d poly3dToPoly4d(const Polytope& poly3d, double x){
    Polytope4d res;
    for(int i=0; i<poly3d.size(); ++i){
      Eigen::Vector3d v3d = poly3d[i];
      Eigen::Vector4d v4d(v3d.data());
      v4d(3) = x;
      res.push_back(v4d);
    }
    return res;
  }

  // return bounds generated from lb to ub
  iris::Polyhedron getPolyFromBounds(const Point4d &lb, const Point4d &ub){
    //cout << Eigen::MatrixXd::Identity(4,4) << endl;
    Eigen::MatrixXd A(8,4);
    A << Eigen::MatrixXd::Identity(4,4), -Eigen::MatrixXd::Identity(4,4);
    Eigen::VectorXd B(8);
    B << ub, -lb;
    iris::Polyhedron res(A, B);
    //cout << "A" << endl
    //  << res.getA() << endl;
    //cout << "B" << endl
    //  << res.getB() << endl;
    return res;
  }

  Point getCentroid(const Polytope &uav){
    Point res(0,0,0);
    for(int i=0; i<uav.size(); ++i){
      res += uav[i];
    }
    res /= uav.size();
    return res;
  }

  double getMinInterDis(const vector<Polytope> &uavs){
    // compute centroids of each uav
    vector<Point> centroids;
    for(int i=0; i<uavs.size(); ++i){
      Polytope uav = uavs[i];
      Point p = getCentroid(uav);
      centroids.push_back(p);
    }

    // compute min distance between centroids
    double res = INT_MAX;
    for(int i=0; i<centroids.size(); ++i){
      for(int j=i; j<centroids.size(); ++j){
        Point s = centroids[i];
        Point t = centroids[j];
        double disSquare = (s-t).transpose() * (s-t);
        res = min(res, disSquare);
      }
    }
    res = sqrt(res);
    return res;
  }

  double getRadius(const Polytope &uav){
    double res = -1;
    Point centroid = getCentroid(uav);
    for(int i=0; i<uav.size(); ++i){
      Point p = uav[i];
      double disSquare = (p-centroid).transpose() * (p-centroid);
      res = max(res, disSquare);
    }
    res = sqrt(res);
    return res;
  }

  Eigen::MatrixXd quatRotateVecDiff(const Eigen::Vector4d &q, const Eigen::Vector3d &v){
    //dq
    Eigen::MatrixXd dq(4,7);
    dq << Eigen::MatrixXd::Identity(4,4), Eigen::MatrixXd::Zero(4,3);
    //dr
    Eigen::MatrixXd dr(4,7);
    Eigen::MatrixXd auxMat(4,3);
    auxMat << Eigen::MatrixXd::Zero(1,3), Eigen::MatrixXd::Identity(3,3);
    dr << Eigen::MatrixXd::Zero(4,4), auxMat;
    //q_times_r && dq_times_r
    Eigen::Vector4d r_0;
    r_0 << 0, v;
    Eigen::Vector4d q_times_r = drake::math::quatProduct(q, r_0);
    Eigen::MatrixXd dq_times_r = quatProductDiff(q, r_0);
    //q_conj && dq_conj
    Eigen::Vector4d q_conj = drake::math::quatConjugate(q);
    Eigen::MatrixXd dq_conj = quatConjugateDiff(q);
    //r_rot, dr_rot
    Eigen::Vector4d r_rot = drake::math::quatProduct(q_times_r, q_conj);
    Eigen::MatrixXd dr_rot = quatProductDiff(q_times_r, q_conj);
    //dr_rot i.e., res
    auxMat.resize(8,7);
    auxMat << dq,dr;
    auxMat = dq_times_r * auxMat;
    Eigen::MatrixXd auxMat2(8,7);
    auxMat2 << auxMat, dq_conj*dq;
    dr_rot = dr_rot * auxMat2;
    dr_rot = dr_rot.block(1,0,dr_rot.rows()-1,dr_rot.cols());
    //to do

    return dr_rot;
  }

  Eigen::MatrixXd quatProductDiff(const Eigen::Vector4d &lhs, const Eigen::Vector4d &rhs){
    Eigen::MatrixXd res;

    //to do

    return res;
  }

  Eigen::MatrixXd quatConjugateDiff(const Eigen::Vector4d &q){
    Eigen::MatrixXd res;

    return res;
  }



}

