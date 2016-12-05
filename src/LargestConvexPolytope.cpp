#include "LargestConvexPolytope.h"
#include <iostream>
using namespace std;

LargestConvexPolytope::Polytope LargestConvexPolytope::minkowskiSum(const Polytope &lhs, const Polytope &rhs) const{
  Polytope res;
  for(int i=0; i<lhs.size(); ++i){
    for(int j=0; j<rhs.size(); ++j){
      Point p1 = lhs[i], p2 = rhs[j];
      res.push_back(p1+p2);
    }
  }
  return res;
}

// return (O+V)x[0,TimeInterval] U Union(U(D+V)xt)
vector<LargestConvexPolytope::Polytope4d> LargestConvexPolytope::getObstacleBar(){
  // O+V x [0,TimeInterval]
  vector<Polytope4d> dilatedStaticObstacles4d;
  for(int i=0; i<staticObstacles_.size(); ++i){
    Polytope obs = staticObstacles_[i];
    for(int j=0; j<uavShapes_.size(); ++j){
      // O+V
      Polytope uavShape = uavShapes_[j];
      Polytope dilatedObs = minkowskiSum(obs, uavShape);
      // O+V x [0, TimeInterval]
      Polytope4d dilatedObsCrossZero4d = poly3dToPoly4d(dilatedObs, 0);
      Polytope4d dilatedObsCrossTimeInterval4d = poly3dToPoly4d(dilatedObs, timeInterval_);
      Polytope4d concat4d = dilatedObsCrossZero4d;
      concat4d.insert(concat4d.end(), dilatedObsCrossTimeInterval4d.begin(), dilatedObsCrossTimeInterval4d.end());
      dilatedStaticObstacles4d.push_back(concat4d);
    }
  }

  // D+V x t : t c [0, TimeInterval]
  vector<Polytope4d> dilatedDynamicObstacles4d;
  for(int i=0; i<dynamicObstacles_.size(); ++i){
    Polytope obs = dynamicObstacles_[i];
    for(int j=0; j<uavShapes_.size(); ++j){
      Polytope uavShape = uavShapes_[j];
      // to be confirmed
      Polytope polyBeforeMove = polytopeAfterMove(obs, currTime_, dynamicObstaclesTrajectories_[i]);

      Polytope polyAfterMove = polytopeAfterMove(obs, currTime_+timeInterval_, dynamicObstaclesTrajectories_[i]);
      Polytope dilatedPolyBeforeMove = minkowskiSum(polyBeforeMove, uavShape);
      Polytope dilatedPolyAfterMove = minkowskiSum(polyAfterMove, uavShape);
      Polytope4d dilatedPolyBeforeMove4d = poly3dToPoly4d(dilatedPolyBeforeMove, 0);
      Polytope4d dilatedPolyAfterMove4d = poly3dToPoly4d(dilatedPolyAfterMove, timeInterval_);
      Polytope4d concat4d = dilatedPolyBeforeMove4d;
      concat4d.insert(concat4d.end(), dilatedPolyAfterMove4d.begin(), dilatedPolyAfterMove4d.end());
      dilatedDynamicObstacles4d.push_back(concat4d);
    }
  }

  vector<Polytope4d> res;
  res = dilatedStaticObstacles4d;
  res.insert(res.end(), dilatedDynamicObstacles4d.begin(), dilatedDynamicObstacles4d.end());
  return res;
}

// return polytope after moving
LargestConvexPolytope::Polytope LargestConvexPolytope::polytopeAfterMove(const Polytope &poly, double time, trajectory m) const {
  Polytope res;
  for(int i=0; i<poly.size(); ++i){
    Point p = poly[i];
    Point pAfterMove = (*m)(p, time);
    res.push_back(pAfterMove);
  }
  return res;
}

// return poly4d appending x to each vertice in poly3d
LargestConvexPolytope::Polytope4d LargestConvexPolytope::poly3dToPoly4d(const Polytope& poly3d, double x) const{
  Polytope4d res;
  for(int i=0; i<poly3d.size(); ++i){
    Eigen::Vector3d v3d = poly3d[i];
    Eigen::Vector4d v4d(v3d.data());
    v4d(3) = x;
    res.push_back(v4d);
  }
  return res;
}

const int INFI = INT_MAX;
//const int INFI = 5;

// return true if valid convex polytope, false otherwise
// region of directed largest convex region in free space is return through param
bool LargestConvexPolytope::directedLargestConvexRegionInFreeSpace(iris::IRISRegion &_region, const vector<Point4d> &in, const vector<Polytope4d> &obstacles) const {

  // format the output so as to visualize in jupyter conveniently
  Eigen::IOFormat np_array(Eigen::StreamPrecision, 0, ", ", ",\n", "[", "]", "np.array([", "])");

  //problem:
  //    obstacle_pts
  //    bounds
  //    dim
  //    seed

  // dim
  iris::IRISProblem problem(4);

  // seed
  Eigen::Vector4d avgIn;
  if(in.size()>0){
    for(int i=0; i<in.size(); ++i){
      avgIn += in[i];
    }
    avgIn /= in.size();
  }
  else{
    avgIn << 0.1,0.1,0.1,0.1;
    //avgIn << 10,10,10,0;
  }
  problem.setSeedPoint(avgIn);
#ifdef DEBUG
  cout << "seed" << endl
    << problem.getSeed().getC().format(np_array) << endl
    << problem.getSeed().getD().format(np_array) << endl;
#endif

  // obs: dim * num of points
  for(int i=0; i<obstacles.size(); ++i){
    Polytope4d poly4d = obstacles[i];
    Eigen::MatrixXd obs(poly4d[0].size(), poly4d.size());
    for(int j=0; j<poly4d.size(); ++j){
      obs.col(j) = poly4d[j];
    }
    problem.addObstacle(obs);
#ifdef DEBUG
    if(i==0) cout << "obstacles:" << endl;
    cout << obs << endl;
#endif
  }

  // bounds zc[0,TimeInterval]
  Point4d lb, ub;

#ifdef VERSION_3D
  //3d version
  lb << -INFI, -INFI, -1e-20, 0;
  ub << INFI, INFI, 1e-20, timeInterval_;
#else
  //4d version
  lb << -INFI, -INFI, -INFI, 0;
  ub << INFI, INFI, INFI, timeInterval_;
#endif

  iris::Polyhedron bound = getPolyFromBounds(lb, ub);
  problem.setBounds(bound);
#ifdef DEBUG
  cout << "A of bound:" << endl
    << bound.getA().format(np_array) << endl;
  cout << "B of bound:" << endl
    << bound.getB().format(np_array) << endl;
#endif

  // option require_containment
  iris::IRISOptions options;
  if(in.size()>0) {
    //    options.require_containment = true;
    vector<Point4d> requiredPoints = in;
    vector<Eigen::VectorXd> requiredPointsVxd;
    for(int i=0; i<requiredPoints.size(); ++i){
      Eigen::VectorXd v(4);
      v = requiredPoints[i];
      requiredPointsVxd.push_back(v);
    }
    options.set_required_containment_points(requiredPointsVxd);
#ifdef DEBUG
    cout << "requiredPoints:" << endl;
    for(int i=0; i<requiredPointsVxd.size(); ++i){
      cout << requiredPointsVxd[i] << endl;
    }
#endif
  }

  // solve
  iris::IRISRegion region;
  try{
    region = inflate_region(problem, options);
    //stop at the begining due to containment requirement
    if(region.polyhedron.getA().rows()==0) return false;
  }
  catch(iris_mosek::InnerEllipsoidInfeasibleError e){
    //no feasible solution
    return false;
  }
#ifdef DEBUG
  std::cout << "a = " << region.polyhedron.getA().format(np_array) << std::endl;
  std::cout << "b = " << region.polyhedron.getB().format(np_array) << std::endl;

  std::cout << "c = " << region.ellipsoid.getC().format(np_array) << std::endl;
  std::cout << "d = " << region.ellipsoid.getD().format(np_array) << std::endl;

  Eigen::VectorXd vxd(4);
  vxd = in[0];
  cout << "vxd" << endl
    << vxd << endl
    << "is in polyhedron: " << region.polyhedron.contains(vxd, 0 ) << endl;
#endif

  _region = region;
  return true;
}

// return bounds generated from lb to ub
iris::Polyhedron LargestConvexPolytope::getPolyFromBounds(const Point4d &lb, const Point4d &ub) const{
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

// return true if valid convex polytope, false otherwise
// linear constraints is passed through _A and _B
bool LargestConvexPolytope::getLargestConvexPolytope(Eigen::MatrixXd& _A, Eigen::VectorXd& _B){
  vector<Polytope4d> obstacleBar = getObstacleBar();
  iris::IRISRegion region_fo_g, region_o_g;
  bool isValid_fo_g, isValid_o_g;

  vector<Point4d> in;
  Point coor3d;
  Point4d coor4d;

  //P_{fo->g}
  in.clear();
  //[p1, ..., pn] x 0
  for(int i=0; i<uavs_.size(); ++i){
    Polytope uav = uavs_[i];
    for(int j=0; j<uav.size(); ++j){
      coor3d = uav[j];
      coor4d << coor3d, 0;
      in.push_back(coor4d);
    }
  }
  //[gDir, TimeInterval]
  coor4d << gDir_, timeInterval_;
  in.push_back(coor4d);
  isValid_fo_g = directedLargestConvexRegionInFreeSpace(region_fo_g, in, obstacleBar);

  //P_{o->g}
  in.clear();
  //[(p1 + ... + pn)/n, 0]
  coor3d << 0,0,0;
  int uavPointCount = 0;
  for(int i=0; i<uavs_.size(); ++i){
    Polytope uav = uavs_[i];
    for(int j=0; j<uav.size(); ++j){
      coor3d += uav[j];
      ++uavPointCount;
    }
  }
  coor3d /= uavPointCount;
  coor4d << coor3d, 0;
  in.push_back(coor4d);
  //[gDir, TimeInterval]
  coor4d << gDir_, timeInterval_;
  in.push_back(coor4d);
  isValid_o_g = directedLargestConvexRegionInFreeSpace(region_o_g, in, obstacleBar);

  //satisfy both i and ii
  if(isValid_fo_g && isValid_o_g){
    Eigen::MatrixXd A_fo_g = region_fo_g.getPolyhedron().getA();
    Eigen::VectorXd B_fo_g = region_fo_g.getPolyhedron().getB();
    Eigen::MatrixXd A_o_g = region_o_g.getPolyhedron().getA();
    Eigen::VectorXd B_o_g = region_o_g.getPolyhedron().getB();
    _A.resize(A_fo_g.rows()+A_o_g.rows(), A_fo_g.cols());
    _A << A_fo_g, A_o_g;
    _B.resize(B_fo_g.rows()+B_o_g.rows(), B_fo_g.cols());
    _B << B_fo_g, B_o_g;
    return true;
  }
  //only satisfy only i
  if(isValid_fo_g){
    Eigen::MatrixXd A_fo_g = region_fo_g.getPolyhedron().getA();
    Eigen::VectorXd B_fo_g = region_fo_g.getPolyhedron().getB();
    _A = A_fo_g;
    _B = B_fo_g;
    return true;
  }
  //otherwise
  return false;
}
