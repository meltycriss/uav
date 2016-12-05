#include "LargestConvexPolytope.h"
#include <iostream>
#include <string>
using namespace std;

LargestConvexPolytope::Point traj1(LargestConvexPolytope::Point p, double t){
  LargestConvexPolytope::Point res = p;
  LargestConvexPolytope::Point dir(0, -1, 0);
  res += dir * t;
  return res;
}

int main(){
  Eigen::IOFormat np_array(Eigen::StreamPrecision, 0, ", ", ",\n", "[", "]", "np.array([", "])");

  //test getObstacleBar()
  cout << "-----------------getObstacleBar()-----------------" << endl;
  vector<LargestConvexPolytope::Polytope> uavShapes;
  vector<LargestConvexPolytope::Polytope> staticObstacles;
  vector<LargestConvexPolytope::Polytope> dynamicObstacles;
  vector<LargestConvexPolytope::trajectory> dynamicObstaclesTrajectories;
  double timeInterval = 1;
  double currTime = 2;

  LargestConvexPolytope::Polytope uavShape;
  LargestConvexPolytope::Polytope staticObstacle;
  LargestConvexPolytope::Polytope dynamicObstacle;
  LargestConvexPolytope::trajectory dynamicObstaclesTrajectory;

  LargestConvexPolytope::Point p;
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

  vector<LargestConvexPolytope::Polytope4d> vecPoly4d = lcp.getObstacleBar();

  // test directedLargestConvexRegionInFreeSpace()
  cout << "-----------------directedLargestConvexRegionInFreeSpace()-----------------" << endl;
  vector<LargestConvexPolytope::Point> uavs;
  LargestConvexPolytope::Point gDir;

  LargestConvexPolytope::Point uav;
  uav << -1,1,0;
  uavs.push_back(uav);
  uav << -1,-1,0;
  uavs.push_back(uav);
  uav << -3,-1,0;
  uavs.push_back(uav);
  uav << -3,1,0;
  uavs.push_back(uav);

  gDir << -1,-2,0; 

  vector<LargestConvexPolytope::Point4d> vecPoint4d;
  LargestConvexPolytope::Point4d point4d;

  for(int i=0; i<uavs.size(); ++i){
    LargestConvexPolytope::Point p3d = uavs[i];
    LargestConvexPolytope::Point4d p4d(p3d.data());
    vecPoint4d.push_back(p4d);
  }
  point4d << gDir, timeInterval;
  vecPoint4d.push_back(point4d);

  iris::IRISRegion region;
  if(lcp.directedLargestConvexRegionInFreeSpace(region, vecPoint4d, vecPoly4d)){
    cout << "A" << endl 
      << region.polyhedron.getA() << endl;
    cout << "B" << endl 
      << region.polyhedron.getB() << endl;
  }
  else{
    cout << "shoot" << endl;
  }

  //test getLargestConvexPolytope()
  cout << "-----------------getLargestConvexPolytope()-----------------" << endl;
  lcp.setGDir(gDir);
  vector<LargestConvexPolytope::Polytope> vecUavs;
  vecUavs.push_back(uavs);
  lcp.setUavs(vecUavs);
  Eigen::MatrixXd A;
  Eigen::VectorXd B;
  lcp.getLargestConvexPolytope(A, B);
  cout << "A" << endl
    << A.format(np_array) << endl;
  cout << "B" << endl
    << B.format(np_array) << endl;

  return 0;
}
