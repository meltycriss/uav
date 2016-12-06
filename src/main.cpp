#include <Eigen/Core>
#include <iostream>
#include <string>
using namespace std;

int main(){
  Eigen::IOFormat np_array(Eigen::StreamPrecision, 0, ", ", ",\n", "[", "]", "np.array([", "])");

  typedef Eigen::Matrix<double,8,1> Vector8d;

  Vector8d v8d;

  cout << v8d << endl;

  return 0;
}
