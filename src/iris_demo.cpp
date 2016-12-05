#include <iostream>
#include <Eigen/Core>
#include "iris/iris.h"
#include <cmath>

const int INFI = INT_MAX;
using namespace std;

int main(int argc, char** argv) {
  //problem:
  //    obstacle_pts
  //    bounds
  //    dim
  //    seed
  iris::IRISProblem problem(3);
  problem.setSeedPoint(Eigen::Vector3d(0.1, 0.1, 0.1));

  // format the output so as to visualize in jupyter conveniently
  Eigen::IOFormat np_array(Eigen::StreamPrecision, 0, ", ", ",\n", "[", "]", "np.array([", "])");

  // obs: dim * num of points
  Eigen::MatrixXd obs(3,4);

  obs << sqrt(3),     0,     0,    sqrt(3),
               0,     0,     0,          0,
           -INFI, -INFI,  INFI,       INFI;
  problem.addObstacle(obs);
  std::cout << "obs1 = " << obs.format(np_array) << std::endl;

  obs << sqrt(3),     0,     0,    sqrt(3),
               1,     1,     1,          1,
           -INFI, -INFI,  INFI,       INFI;
  problem.addObstacle(obs);
  obs <<       0,     0,     0,          0,
               0,     1,     1,          0,
           -INFI, -INFI,  INFI,       INFI;
  problem.addObstacle(obs);
  obs << sqrt(3), sqrt(3), sqrt(3), sqrt(3),
               0,       1,       1,       0,
           -INFI,   -INFI,    INFI,    INFI;
  problem.addObstacle(obs);
  obs << sqrt(3), sqrt(3), 0, 0,
               0,       1, 1, 0,
               0,       0, 0, 0;
  problem.addObstacle(obs);
  obs << sqrt(3), sqrt(3),     0,     0,
               0,       1,     1,     0,
           3.0/2,   3.0/2, 3.0/2, 3.0/2;
  problem.addObstacle(obs);

  // option require_containment
  iris::IRISOptions options;
  iris::IRISRegion region = inflate_region(problem, options);


  std::cout << "a = " << region.polyhedron.getA().format(np_array) << std::endl;
  std::cout << "b = " << region.polyhedron.getB().format(np_array) << std::endl;

  std::cout << "c = " << region.ellipsoid.getC().format(np_array) << std::endl;
  std::cout << "d = " << region.ellipsoid.getD().format(np_array) << std::endl;
  return 0;
}
