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

  Eigen::Vector4d q(1,2,3,4);
  q = q/q.norm();
  Eigen::Vector3d v(3,4,5);
  cout << v.norm() << endl;
  cout << quatRotateVec(q,v).norm() << endl;


  

  return 0;
}
