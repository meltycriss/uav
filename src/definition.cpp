#include "definition.h"
#include "util.h"
#include <cmath>
using namespace std;

namespace uav{

  Formation::Formation(
      std::vector<Polytope> _uavs,
      Polytope _convexHull,
      double _pref
      ):
    uavs(_uavs),
    convexHull(_convexHull),
    pref(_pref){
      //compute mininterdis and radius
      minInterDis = getMinInterDis(uavs);
      double r = -1;
      for(int i=0; i<uavs.size(); ++i){
        Polytope uav = uavs[i]:
        double currUavRadius = getRadius(uav);
        r = max(r, currUavRadius);
      }
      radius = r;
    }

}

