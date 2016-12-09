#include <Eigen/Core>
#include <iostream>
#include <string>
#include "OptimalFormation.h"
#include <cmath>
#include "drake/math/quaternion.h"

using namespace std;
using namespace uav;
using drake::math::quatRotateVec;

int main(){
  Eigen::IOFormat np_array(Eigen::StreamPrecision, 0, ", ", ",\n", "[", "]", "np.array([", "])");

  Eigen::Vector4d q(1,4,4,6);
  Eigen::Vector3d r(4,5,6);
  q = q/q.norm();
  cout << quatRotateVecDiff(q, r) << endl;



  

  return 0;
}
