#include "LargestConvexPolytope.h"
#include "OptimalFormation.h"
#include <iostream>
#include <string>
#include "drake/math/quaternion.h"
#include "util.h"
#include "iris/geometry.h"
using namespace std;
using namespace uav;

Point traj1(Point p, double t){
  Point res = p;
  Point dir(0, -1, 0);
  res += dir * t;
  return res;
}

int main(){
  Eigen::IOFormat np_array(Eigen::StreamPrecision, 0, ", ", ",\n", "[", "]", "np.array([", "])");

  //test getObstacleBar()
  cout << "-----------------getObstacleBar()-----------------" << endl;
  vector<Polytope> uavShapes;
  vector<Polytope> staticObstacles;
  vector<Polytope> dynamicObstacles;
  vector<trajectory> dynamicObstaclesTrajectories;
  double timeInterval = 1;
  double currTime = 2;

  Polytope uavShape;
  Polytope staticObstacle;
  Polytope dynamicObstacle;
  trajectory dynamicObstaclesTrajectory;

  Point p;
  p << 1,1,0;
  uavShape.push_back(p);
  p << -1,1,0;
  uavShape.push_back(p);
  p << -1,-1,0;
  uavShape.push_back(p);
  p << 1,-1,0;
  uavShape.push_back(p);

  p << 1,1,0;
  staticObstacle.push_back(p);
  p << 3,1,0;
  staticObstacle.push_back(p);
  p << 3,3,0;
  staticObstacle.push_back(p);
  p << 1,3,0;
  staticObstacle.push_back(p);

  p << 3,1,0;
  dynamicObstacle.push_back(p);
  p << 5,1,0;
  dynamicObstacle.push_back(p);
  p << 5,-1,0;
  dynamicObstacle.push_back(p);
  p << 3,-1,0;
  dynamicObstacle.push_back(p);

  dynamicObstaclesTrajectory = &traj1;

  uavShapes.push_back(uavShape);
  staticObstacles.push_back(staticObstacle);
  dynamicObstacles.push_back(dynamicObstacle);
  dynamicObstaclesTrajectories.push_back(dynamicObstaclesTrajectory);

  LargestConvexPolytope lcp = LargestConvexPolytope();
  lcp.setUavShapes(uavShapes);
  lcp.setStaticObstacles(staticObstacles);
  lcp.setDynamicObstacles(dynamicObstacles);
  lcp.setDynamicObstaclesTrajectories(dynamicObstaclesTrajectories);
  lcp.setTimeInterval(timeInterval);
  lcp.setCurrTime(currTime);

//  vector<Polytope4d> vecPoly4d = lcp.getObstacleBar();

  // test directedLargestConvexRegionInFreeSpace()
  cout << "-----------------directedLargestConvexRegionInFreeSpace()-----------------" << endl;
  vector<Point> uav;
  Point gDir;

//  Point p;
  p << -1,1,0;
  uav.push_back(p);
  p << -1,-1,0;
  uav.push_back(p);
  p << -3,-1,0;
  uav.push_back(p);
  p << -3,1,0;
  uav.push_back(p);

  //gDir << -1,-2,0; 
  gDir << 0.1,-1,0; //test scale

  vector<Point4d> vecPoint4d;
  Point4d point4d;

  point4d << gDir, timeInterval;
  vecPoint4d.push_back(point4d);
  for(int i=0; i<uav.size(); ++i){
    Point p3d = uav[i];
    Point4d p4d(p3d.data());
    vecPoint4d.push_back(p4d);
  }

//  iris::IRISRegion region;
//  Eigen::MatrixXd A;
//  Eigen::VectorXd B;
//  if(lcp.directedLargestConvexRegionInFreeSpace(region, vecPoint4d, vecPoly4d)){
//    A = region.polyhedron.getA();
//    B = region.polyhedron.getB();
////    cout << "A" << endl 
////      << region.polyhedron.getA() << endl;
////    cout << "B" << endl 
////      << region.polyhedron.getB() << endl;
//  }
//  else{
//    cout << "shoot" << endl;
//  }

  //test getLargestConvexPolytope()
  cout << "-----------------getLargestConvexPolytope()-----------------" << endl;
  lcp.setGDir(gDir);
  vector<Polytope> uavs;
  uavs.push_back(uav);
  lcp.setUavs(uavs);
  Eigen::MatrixXd A;
  Eigen::VectorXd B;
  lcp.getLargestConvexPolytope(A, B);

  Eigen::MatrixXd disp_A = A;
  Eigen::VectorXd disp_B = B;
  reducePolyDim(disp_A, disp_B, 2);
  cout << "a = "
    << disp_A.format(np_array) << endl;
  cout << "b = "
    << disp_B.format(np_array) << endl;
  cout << "INFI = " << INFI << endl;
  
  iris::Polyhedron poly(A,B);
  cout << "poly contains gDir: "<< poly.contains(Eigen::Vector4d(gDir(0),gDir(1),gDir(2),timeInterval), 0) << endl;



 //test OptimalFormation::optimalDeviation()
 cout << "-----------------optimalDeviation()-----------------" << endl;
 OptimalFormation of;
 Formation formation(uavs, uavShapes, uav, 1);
 //Formation formation(uavs, vector<Point>(1, Point(1,3,5)), 1);
 cout << formation.minInterDis << endl;
 cout << formation.radius << endl;
 vector<Formation> formations;
 formations.push_back(formation);
 of.setFormations(formations);
 OptimalFormation::init(A, B, gDir, 1, Eigen::MatrixXd::Identity(4,1), 1, 1, 1, timeInterval);
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

  


  return 0;
}
