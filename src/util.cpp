#include <iostream>
#include "util.h"
#include <cmath>
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
}

