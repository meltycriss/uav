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

  Eigen::Matrix<double,2,4> mat;
  mat << 1,2,3,4,
      5,6,7,8;
  cout << mat << endl << endl;
  cout << mat.block(0,0,2,3) << endl;
  Eigen::Matrix<double,2,3> mat1 = mat.block(0,0,2,mat.cols()-1);
  Eigen::Matrix<double,2,1> mat2 = mat.block(0,3,2,1);
  Eigen::MatrixXd mat3(2,4);
  mat3 << mat1,mat2;
  cout << "mat1" << endl << mat1 << endl;
  cout << "mat2" << endl << mat2 << endl;
  cout << "mat3" << endl << mat3 << endl;

  Eigen::Vector4d v4d;
  Eigen::Vector3d v3d(1,2,3);
  v4d << v3d,10086;
  cout << v4d << endl;

  Eigen::MatrixXd mat4(3,4);
  Eigen::MatrixXd mat5(1,4);
  mat4 << mat,mat5;
  cout << mat4 << endl;

  cout << Eigen::MatrixXd::Zero(5,5) << endl;



  

  return 0;
}
