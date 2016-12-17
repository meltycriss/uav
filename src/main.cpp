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

  Eigen::MatrixXd mat(5,5);
  mat << 9.5,11.5,14.5,11.5,7.5,
      6.5,15.5,13.5,13.5,10.5,
      12.5,13.5,6.5,8.5,8.5,
      11.5,9.5,10.5,12.5,9.5,
      7.5,12.5,14.5,10.5,14.5;
  hungarian(mat);
  return 0;
}
