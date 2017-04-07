#include "util.h"
#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;
using namespace uav;

int main(){
  vector<pair<double, double> > src, dest;
  src.push_back(make_pair(0, 10));
  src.push_back(make_pair(-5, 0));
  src.push_back(make_pair(5, 0));
  dest.push_back(make_pair(0, 30));
  dest.push_back(make_pair(-7, 18));
  dest.push_back(make_pair(7, 20));
//  for(int i=0; i<src.size(); ++i){
//    pair<double, double> p = src[i];
//    p.second += 10;
//    dest.push_back(p);
//  }
  vector<int> vecAssigned;
  for(int i=0; i<3; ++i){
    vecAssigned.push_back(i);
  }
  do{
    for(int i=0; i<vecAssigned.size(); ++i){
      cout << vecAssigned[i] << " ";
    }
    cout << endl;
    double sum = 0;
    for(int i=0; i<src.size(); ++i){
      pair<double, double> posSrc = src[i];
      pair<double, double> posDest = dest[vecAssigned[i]];
      double x = posSrc.first - posDest.first;
      double y = posSrc.second- posDest.second;
      double distSquare = x*x + y*y;
      sum += distSquare;
      cout << vecAssigned[i] << ": " << distSquare << endl;
    }
    cout << "sum: " << sum << endl;
  }while(next_permutation(vecAssigned.begin(), vecAssigned.end()));

  Eigen::MatrixXd matCost(src.size(), src.size());
  for(int i=0; i<matCost.rows(); ++i){
    pair<double, double> posSrc = src[i];
    for(int j=0; j<matCost.cols(); ++j){
      pair<double, double> posDest = dest[j];
      double x = posSrc.first - posDest.first;
      double y = posSrc.second- posDest.second;
      double distSquare = x*x + y*y;
      matCost(i,j) = distSquare;
    }
  }
  cout << "matCost: " << endl;
  cout << matCost << endl;
  cout << "Hungarian res: ";
  vector<int> res = hungarian(matCost);
  for(int i=0; i<res.size(); ++i){
    cout << res[i] << " ";
  }
  cout << endl;


  return 0;

}
