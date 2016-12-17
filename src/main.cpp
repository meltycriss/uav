#include <iostream>
#include <Eigen/Core>
#include "iris/iris.h"
#include <cmath>
#include <limits.h>

#include <stdio.h>
#include <string.h>
#include <iostream>

#include "sntoyA.hpp"
#include "snoptProblem.hpp"

#include "LargestConvexPolytope.h"
#include "OptimalFormation.h"
#include <iostream>
#include <string>
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

  Eigen::MatrixXd mat(3,3);
  mat << 1,2,3,
      4,5,6,
      7,8,9;
  mat.row(0) = vecAdd(mat.row(1), mat(1, minOfVec(mat.row(1))));
  cout << mat << endl;
  return 0;
}
