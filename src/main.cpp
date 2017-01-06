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
#include <stdlib.h>
#include <time.h>
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

  Eigen::Vector4d v4d(0,0,3,4);
  srand(time(NULL));
  for(int i=0; i<1; ++i){
    cout << (rand() % 4 + 1) * Eigen::MatrixXd::Ones(4,1) << endl;
  }


  return 0;
}
