#include <iostream>
#include "util.h"
#include <cmath>
#include "drake/math/quaternion.h"
#include <exception>
#include <set>
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
    Point res(0.0,0.0,0.0);
    for(int i=0; i<uav.size(); ++i){
      res = res + uav[i];
    }
    res = res / uav.size();
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
      for(int j=i+1; j<centroids.size(); ++j){
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
    Eigen::MatrixXd res = dr_rot.block(1,0,dr_rot.rows()-1,dr_rot.cols());
    return res;
  }

  Eigen::MatrixXd quatProductDiff(const Eigen::Vector4d &lhs, const Eigen::Vector4d &rhs){
    //w1
    double w1 = lhs(0);
    //w2
    double w2 = rhs(0);
    //v1
    Eigen::Vector3d v1 = lhs.block(1,0,3,1);
    //v2
    Eigen::Vector3d v2 = rhs.block(1,0,3,1);

    //dq1
    Eigen::MatrixXd dq1(4,8);
    dq1 << Eigen::MatrixXd::Identity(4,4), Eigen::MatrixXd::Zero(4,4);
    //dq2
    Eigen::MatrixXd dq2(4,8);
    dq2 << Eigen::MatrixXd::Zero(4,4), Eigen::MatrixXd::Identity(4,4);
    //dw1
    Eigen::MatrixXd dw1 = dq1.block(0,0,1,dq1.cols());
    //dw2
    Eigen::MatrixXd dw2 = dq2.block(0,0,1,dq2.cols());
    //dv1
    Eigen::MatrixXd dv1 = dq1.block(1,0,dq1.rows()-1,dq1.cols());
    //dv2
    Eigen::MatrixXd dv2 = dq2.block(1,0,dq2.rows()-1,dq2.cols());

    //dw3
    Eigen::MatrixXd dw3 = w2*dw1 + w1*dw2 - v2.transpose()*dv1 - v1.transpose()*dv2;
    //dv3
    Eigen::MatrixXd dv3 = dcross(v1,v2,dv1,dv2) + v2*dw1 + w1*dv2 + v1*dw2 + w2*dv1;

    //dq3
    Eigen::MatrixXd dq3(4,8);
    dq3 << dw3, dv3;

    return dq3;
  }

  Eigen::MatrixXd quatConjugateDiff(const Eigen::Vector4d &q){
    Eigen::MatrixXd res = -Eigen::MatrixXd::Identity(4,4);
    res(0,0) = 1;
    return res;
  }

  Eigen::MatrixXd dcross(const Eigen::Vector3d &r1, const Eigen::Vector3d &r2, Eigen::MatrixXd &dr1, Eigen::MatrixXd &dr2){
    Eigen::MatrixXd lhs(3,8), rhs(3,8);
    for(int i=0; i<8; ++i){
      Eigen::Vector3d v3d = dr2.col(i);
      lhs.col(i) = r1.cross(v3d);
      v3d = dr1.col(i);
      rhs.col(i) = r2.cross(v3d);
    }
    return lhs-rhs;
  }

  void reducePolyDim(Eigen::MatrixXd &A, Eigen::VectorXd &b, int dim){
    removeColumn(A, dim);
    for(int i=0; i<A.rows(); ++i){
      bool allZero = true;
      for(int j=0; j<A.cols(); ++j){
        if(A(i,j)!=0){
          allZero = false;
          break;
        }
      }
      if(allZero){
        removeRow(A, i);
        removeRow(b, i);
        --i;
      }
    }
  }

  void removeRow(Eigen::MatrixXd& matrix, unsigned int rowToRemove){
    unsigned int numRows = matrix.rows()-1;
    unsigned int numCols = matrix.cols();

    if( rowToRemove < numRows )
      matrix.block(rowToRemove,0,numRows-rowToRemove,numCols) = matrix.block(rowToRemove+1,0,numRows-rowToRemove,numCols);

    matrix.conservativeResize(numRows,numCols);
  }

  void removeRow(Eigen::VectorXd& vec, unsigned int rowToRemove){
    for(int i=rowToRemove; i<vec.rows()-1; ++i){
      vec(i) = vec(i+1);
    }
    vec.conservativeResize(vec.rows()-1,1);
  }

  void removeColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove){
    unsigned int numRows = matrix.rows();
    unsigned int numCols = matrix.cols()-1;

    if( colToRemove < numCols )
      matrix.block(0,colToRemove,numRows,numCols-colToRemove) = matrix.block(0,colToRemove+1,numRows,numCols-colToRemove);

    matrix.conservativeResize(numRows,numCols);
  }

  vector<int> hungarian(const Eigen::MatrixXd &matCost){

    //must be square matrix
    if(matCost.rows()!=matCost.cols()){
      throw exception();
    }
    Eigen::MatrixXd cost = matCost;
    int rows = cost.rows();
    int cols = cost.cols();

    //res[i]=j indicate row i is assigned to col j
    vector<int> res(rows, -1);
    //number of assigend rows
    int numAssigned = 0;

    //row reduction
    for(int i=0; i<rows; ++i){
      int minEleIdx = minOfVec(cost.row(i));
      double minEle = cost(i, minEleIdx);
      cost.row(i) = vecAdd(cost.row(i), -minEle);
    }
    //col reduction
    for(int i=0; i<cols; ++i){
      int minEleIdx = minOfVec(cost.col(i));
      double minEle = cost(minEleIdx, i);
      cost.col(i) = vecAdd(cost.col(i), -minEle);
    }
    while(true){
      res = vector<int>(rows, -1);
      numAssigned = 0;
      set<int> deletedRows, deletedCols;
      //row scanning
      for(int i=0; i<rows; ++i){
        vector<int> zeroColIdx;
        for(int j=0; j<cols; ++j){
          if(cost(i,j)==0 && deletedRows.count(i)==0 && deletedCols.count(j)==0){
            zeroColIdx.push_back(j);
          }
        }
        if(zeroColIdx.size()==1){
          //line
          deletedCols.insert(zeroColIdx[0]);
          //marked
          res[i] = zeroColIdx[0];
          ++numAssigned;
        }
      }
      //col scanning
      for(int i=0; i<cols; ++i){
        vector<int> zeroRowIdx;
        for(int j=0; j<rows; ++j){
          if(cost(j,i)==0 && deletedRows.count(j)==0 && deletedCols.count(i)==0){
            zeroRowIdx.push_back(j);
          }
        }
        if(zeroRowIdx.size()==1){
          //line
          deletedRows.insert(zeroRowIdx[0]);
          //marked
          res[zeroRowIdx[0]] = i;
          ++numAssigned;
        }
      }
      //case 1
      if(numAssigned==rows){
        break;
      }
      //case 2
      //find min undeleted
      double minUndeleted = INFI;
      for(int i=0; i<rows; ++i){
        for(int j=0; j<cols; ++j){
          //undeleted
          if(deletedRows.count(i)==0 && deletedCols.count(j)==0){
            //min
            if(cost(i,j) < minUndeleted){
              minUndeleted = cost(i,j);
            }
          }
        }
      }
      //update cost matrix
      for(int i=0; i<rows; ++i){
        for(int j=0; j<cols; ++j){
          //intersecion
          if(deletedRows.count(i)>0 && deletedCols.count(j)>0){
            cost(i,j) += minUndeleted;
          }
          //undeleted
          if(deletedRows.count(i)==0 && deletedCols.count(j)==0){
            cost(i,j) -= minUndeleted;
          }
        }
      }
    }

    return res;
  }

  int minOfVec(const Eigen::VectorXd &vec){
    int res;
    double min = INFI;
    for(int i=0; i<vec.size(); ++i){
      if(vec(i)<min){
        min = vec(i);
        res = i;
      }
    }
    return res;
  }

  Eigen::VectorXd vecAdd(const Eigen::VectorXd &vec, double x){
    Eigen::VectorXd res = vec;
    for(int i=0; i<vec.size(); ++i){
      res(i) += x;
    }
    return res;
  }

  //abs coordinates to relative coordinates respect to centroid
  vector<Polytope> absToRela(const vector<Polytope> &abs, const Point &centroid){
    vector<Polytope> res(abs.size());
    for(int i=0; i<abs.size(); ++i){
      Polytope poly = abs[i];
      for(int j=0; j<poly.size(); ++j){
        poly[j] -= centroid;
      }
      res[i] = poly;
    }
    return res;
  }

  //relative coordinates to absolute coordinates respect to centroid
  vector<Polytope> relaToAbs(const vector<Polytope> &rela, const Point &centroid){
    vector<Polytope> res(rela.size());
    for(int i=0; i<rela.size(); ++i){
      Polytope poly = rela[i];
      for(int j=0; j<poly.size(); ++j){
        poly[j] += centroid;
      }
      res[i] = poly;
    }
    return res;
  }

  //relative coordinates to absolute coordinates respect to centroid
  vector<Point> relaToAbs(const vector<Point> &rela, const Point &centroid){
    vector<Point> res(rela.size());
    for(int i=0; i<rela.size(); ++i){
      Point p = rela[i];
      p += centroid;
      res[i] = p;
    }
    return res;
  }

  //move polys with tsq param
  vector<Polytope> tsqTransPolyVec(const vector<Polytope> &polys, const Vector8d &tsq){
    vector<Polytope> res(polys.size());
    for(int i=0; i<polys.size(); ++i){
      Polytope poly = polys[i];
      for(int j=0; j<poly.size(); ++j){
        poly[j] = tsqTransPoint(poly[j], tsq);
      }
      res[i] = poly;
    }
    return res;
  }

  //move a point with tsq param
  Point tsqTransPoint(const Point &p, const Vector8d &tsq){
    Point res;
    Eigen::Vector3d t(tsq(0), tsq(1), tsq(2));
    double s = tsq(3);
    Eigen::Vector4d q(tsq(4), tsq(5), tsq(6), tsq(7));
    res = t + s * drake::math::quatRotateVec(q, p);
    return res;
  }

  //return centroid distance matrix
  Eigen::MatrixXd getDisMat(const vector<Polytope> &lhs, const vector<Polytope> &rhs){
    Eigen::MatrixXd res(lhs.size(), rhs.size());
    for(int i=0; i<res.rows(); ++i){
      Polytope lhsPoly = lhs[i];
      Point lhsCentroid = getCentroid(lhsPoly);
      for(int j=0; j<res.cols(); ++j){
        Polytope rhsPoly = rhs[j];
        Point rhsCentroid = getCentroid(rhsPoly);
        res(i,j) = (lhsCentroid - rhsCentroid).transpose() * (lhsCentroid - rhsCentroid);
        res(i,j) = sqrt(res(i,j));
      }
    }
    return res;
  }

  //return centroid distance matrix
  Eigen::MatrixXd getDisMat(const vector<Polytope> &lhs, const vector<Point> &rhs){
    Eigen::MatrixXd res(lhs.size(), rhs.size());
    for(int i=0; i<res.rows(); ++i){
      Polytope lhsPoly = lhs[i];
      Point lhsCentroid = getCentroid(lhsPoly);
      for(int j=0; j<res.cols(); ++j){
        Point rhsCentroid = rhs[j];
        res(i,j) = (lhsCentroid - rhsCentroid).transpose() * (lhsCentroid - rhsCentroid);
        res(i,j) = sqrt(res(i,j));
      }
    }
    return res;
  }

  //move uavs centered with centroid to uavs centered with goal
  Polytope uavMoveTo(const Point &goal, const Polytope &uav){
    Polytope res = uav;
    Point t = goal - getCentroid(uav);
    for(int i=0; i<res.size(); ++i){
      res[i] += t;
    }
    return res;
  }

}
