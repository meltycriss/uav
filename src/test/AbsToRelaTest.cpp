#include "LargestConvexPolytope.h"
#include "OptimalFormation.h"
#include <iostream>
#include <string>
#include "drake/math/quaternion.h"
#include "util.h"
#include "iris/geometry.h"
#include <cmath>
using namespace std;
using namespace uav;

Point traj1(Point p, double t){
  Point res = p;
  Point dir(0, -1, 0);
  int auxT = floor(t);
  t = t - (auxT/18)*18;
  t = t<9 ? t:18-t;
  res += dir * t;
  return res;
}

int main(){
  Eigen::IOFormat np_array(Eigen::StreamPrecision, 0, ", ", ",\n", "[", "]", "np.array([", "])");

  //aux var
  Point p;

  //gDir
  Point gDir;
  gDir << 14,10,0;
  gDir << 3.5,10,0;

  //uavs
  vector<Polytope> uavs;
  Polytope uav;
  //uav0
  p << -2,2,0;
  uav.push_back(p);
  uavs.push_back(uav);
  //uav1
  p << 2,2,0;
  uav.push_back(p);
  uavs.push_back(uav);
  //uav2
  p << 2,-2,0;
  uav.push_back(p);
  uavs.push_back(uav);
  //uav3
  p << -2,-2,0;
  uav.push_back(p);
  uavs.push_back(uav);

  //uavShapes
  vector<Polytope> uavShapes;
  Polytope uavShape;
  //us0
  p << -1,1,0;
  uavShape.push_back(p);
  p << 1,1,0;
  uavShape.push_back(p);
  p << 1,-1,0;
  uavShape.push_back(p);
  p << -1,-1,0;
  uavShape.push_back(p);
  uavShapes.push_back(uavShape);

  //convex hull of formation
  Polytope convexHull;
  p << -3,3,0;
  convexHull.push_back(p);
  p << 3,3,0;
  convexHull.push_back(p);
  p << 3,-3,0;
  convexHull.push_back(p);
  p << -3,-3,0;
  convexHull.push_back(p);

  //template formation
  vector<Formation> formations;
  Formation formation(uavs, uavShapes, convexHull, 1);
  formations.push_back(formation);

  //staticObstacles
  vector<Polytope> staticObstacles;
  Polytope staticObstacle;
  //so0
  p << 5,4,0;
  staticObstacle.push_back(p);
  p << 8,4,0;
  staticObstacle.push_back(p);
  p << 8,1,0;
  staticObstacle.push_back(p);
  p << 5,1,0;
  staticObstacle.push_back(p);
  staticObstacles.push_back(staticObstacle);

  //dynamicObstacles and corresponding trajectory function
  vector<Polytope> dynamicObstacles;
  vector<trajectory> dynamicObstaclesTrajectories;
  //do0
  Polytope dynamicObstacle;
  trajectory dynamicObstaclesTrajectory;
  p << 10,10,0;
  dynamicObstacle.push_back(p);
  p << 11,10,0;
  dynamicObstacle.push_back(p);
  p << 11,9,0;
  dynamicObstacle.push_back(p);
  p << 10,9,0;
  dynamicObstacle.push_back(p);
  dynamicObstaclesTrajectory = &traj1;
  dynamicObstacles.push_back(dynamicObstacle);
  dynamicObstaclesTrajectories.push_back(dynamicObstaclesTrajectory);

  //timeInterval
  double timeInterval = 1;

  //currTime
  double currTime = 2;

  //weight of optimization cost
  double wT = 1;
  double wS = 1;
  double wQ = 1;

  //prefered param
  double sPref = 1;
  Eigen::Vector4d qPref = Eigen::Vector4d(1,0,0,0);

  //get lcp
  LargestConvexPolytope lcp(
      gDir,
      uavs,
      uavShapes,
      staticObstacles,
      dynamicObstacles,
      timeInterval,
      currTime,
      dynamicObstaclesTrajectories);

  Eigen::MatrixXd lcpA;
  Eigen::VectorXd lcpB;
  bool shouldFormation = lcp.getLargestConvexPolytope(lcpA, lcpB);
  //whether collision free lcp exists
  if(shouldFormation){
    //debug info
    Eigen::MatrixXd disp_A = lcpA;
    Eigen::VectorXd disp_B = lcpB;
    reducePolyDim(disp_A, disp_B, 2);
    cout << "a = "
      << disp_A.format(np_array) << endl;
    cout << "b = "
      << disp_B.format(np_array) << endl;
    cout << "INFI = " << INFI << endl;
    iris::Polyhedron poly(lcpA,lcpB);
    cout << "poly contains gDir: "<< poly.contains(Eigen::Vector4d(gDir(0),gDir(1),gDir(2),timeInterval), 0) << endl;

    //optimal deviation
    OptimalFormation of(formations);
    OptimalFormation::init(lcpA, lcpB, gDir, sPref, qPref, wT, wS, wQ, timeInterval);
    Vector8d param;
    int index = of.optimalFormation(param);

    cout << "index: " << index << endl;
    cout << "param: " << endl << param << endl;
    cout << "formation: " << endl;
    Eigen::Vector3d t(param(0), param(1), param(2));
    double s = param(3);
    Eigen::Vector4d q(param(4), param(5), param(6), param(7));
    Eigen::Vector3d centroid = getCentroid(uav);
    cout << "centroid after transformation:" << endl << t + s * drake::math::quatRotateVec(q, Eigen::Vector3d(0,0,0)) << endl;
    for(int i=0; i<formation.convexHull.size(); ++i){
      Point p = formation.convexHull[i];
      cout << "before: " << endl << p << endl;
      cout << "after: " << endl << t + s * drake::math::quatRotateVec(q, p) << endl;
    }




  }
  else{
    cout << "no CA formation" << endl;
  }



  //Formation formation(uavs, vector<Point>(1, Point(1,3,5)), 1);




  return 0;
}
