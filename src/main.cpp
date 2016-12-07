#include <Eigen/Core>
#include <iostream>
#include <string>
#include "OptimalFormation.h"
#include <cmath>
using namespace std;
using namespace uav;

int main(){
  Eigen::IOFormat np_array(Eigen::StreamPrecision, 0, ", ", ",\n", "[", "]", "np.array([", "])");

  typedef Eigen::Matrix<double,8,1> Vector8d;

  Vector8d v8d;

  cout << v8d << endl;

  //OptimalFormation of;
  //Formation f;
  //of.optimalDeviation(f);
  //
  Eigen::Vector3d s(1,2,3);
  Eigen::Vector3d t(2,6,6);
  cout << sqrt((s-t).transpose() * (s-t)) << endl;

  return 0;
}
