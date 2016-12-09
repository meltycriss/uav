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

  Eigen::MatrixXd mat(4,7);
  Eigen::MatrixXd auxMat(4,3);
  auxMat << Eigen::MatrixXd::Zero(1,3), Eigen::MatrixXd::Identity(3,3);
  mat << Eigen::MatrixXd::Zero(4,4), auxMat;
  cout << mat << endl;
  mat.resize(1,2);
  mat << 1,2;
  mat = mat * Eigen::MatrixXd::Ones(2,1);
  cout << mat << endl;



  

  return 0;
}
