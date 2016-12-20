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

//current uavs centroid
Point currCentroid = Point(0,0,0);

Point traj1(Point p, double t){
  Point res = p;
  Point dir(0, -1, 0);
  int auxT = floor(t);
  t = t - (auxT/18)*18;
  t = t<9 ? t:18-t;
  res += dir * t;
  //relative coordinate
  res -= currCentroid;
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
  gDir << 0,10,0;

  //uavs
  vector<Polytope> uavs;
  Polytope uav;
  //uav0
  p << -2,2,0;
  uav.clear();
  uav.push_back(p);
  uavs.push_back(uav);
  //uav1
  p << 2,2,0;
  uav.clear();
  uav.push_back(p);
  uavs.push_back(uav);
  //uav2
  p << 2,-2,0;
  uav.clear();
  uav.push_back(p);
  uavs.push_back(uav);
  //uav3
  p << -2,-2,0;
  uav.clear();
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
  //caution: uavs in formation is not necessarily the same as actual uavs
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
  double currTime = 0;

  //weight of optimization cost
  double wT = 1;
  double wS = 1;
  double wQ = 1;

  //prefered param
  double sPref = 1;
  Eigen::Vector4d qPref = Eigen::Vector4d(1,0,0,0);

  //looping
  while(currTime<2){
    //translating absolute coordinates to relative coordinates
    Point gDirRela = gDir - currCentroid;
    vector<Polytope> uavsRela = absToRela(uavs, currCentroid);
    vector<Polytope> staticObstaclesRela = absToRela(staticObstacles, currCentroid);
    //relative coordinates of dynamicObstacles is handled by its trajectory funciton

    //solving the problem under relative coordinates
    //get lcp
    LargestConvexPolytope lcp(
        gDirRela,
        uavsRela,
        uavShapes,
        staticObstaclesRela,
        dynamicObstacles,
        timeInterval,
        currTime,
        dynamicObstaclesTrajectories);

    Eigen::MatrixXd lcpA;
    Eigen::VectorXd lcpB;
    bool shouldFormation = lcp.getLargestConvexPolytope(lcpA, lcpB);

    //whether collision free lcp exists
    if(shouldFormation){
      //largest convex polytope debug info
      Eigen::MatrixXd disp_A = lcpA;
      Eigen::VectorXd disp_B = lcpB;
      reducePolyDim(disp_A, disp_B, 2);
      cout << "a = " << disp_A.format(np_array) << endl;
      cout << "b = " << disp_B.format(np_array) << endl;
      cout << "INFI = " << INFI << endl;

      //optimal deviation
      OptimalFormation of(formations);
      OptimalFormation::init(lcpA, lcpB, gDirRela, sPref, qPref, wT, wS, wQ, timeInterval);
      Vector8d optimalParam;
      int index = of.optimalFormation(optimalParam);

      //update absolute coordinate according to optimal deviation
      
      cout << "-----------------------------------------------------------" << endl;
      cout << "gDir: " << endl << gDirRela + currCentroid << endl;

      cout << "-----------------------------------------------------------" << endl;
      cout << "centroidBefore:" << endl << currCentroid << endl;
      cout << "-----------------------------------------------------------" << endl;
      for(int i=0; i<uavs.size(); ++i){
        Polytope poly = uavs[i];
        cout << "uavBefore" << i << endl;
        for(int j=0; j<poly.size(); ++j){
          cout << poly[j] << endl;
        }
      }

      //uavs
      uavsRela = tsqTransPolyVec(formations[index].uavs, optimalParam);
      uavs = relaToAbs(uavsRela, currCentroid);
      //matching
      

      cout << "-----------------------------------------------------------" << endl;
      cout << "centroidAfter:" << endl << tsqTransPoint(currCentroid, optimalParam) << endl;
      cout << "-----------------------------------------------------------" << endl;
      for(int i=0; i<uavs.size(); ++i){
        Polytope poly = uavs[i];
        cout << "uavAfter" << i << endl;
        for(int j=0; j<poly.size(); ++j){
          cout << poly[j] << endl;
        }
      }

      //update currCentroid
      currCentroid = Point(0,0,0);
      for(int i=0; i<uavs.size(); ++i){
        currCentroid += getCentroid(uavs[i]);
      }
      currCentroid /= uavs.size();

      //timer
      ++currTime;

      gDir << 3,10,0;
    }
    else{
      cout << "no CA formation" << endl;
      ++currTime;
    }

  }

  return 0;
}
