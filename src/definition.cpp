#include "definition.h"
#include "util.h"
#include <cmath>
using namespace std;

namespace uav{

  Formation::Formation(
      std::vector<Polytope> _uavs,
      std::vector<Polytope> _uavShapes,
      Polytope _convexHull,
      double _pref
      ):
    uavs(_uavs),
    uavShapes(_uavShapes),
    convexHull(_convexHull),
    pref(_pref){
      //compute mininterdis and radius
      minInterDis = getMinInterDis(uavs);
      double r = -1;
      for(int i=0; i<uavShapes.size(); ++i){
        Polytope uavShape = uavShapes[i];
        double currUavRadius = getRadius(uavShape);
        r = max(r, currUavRadius);
      }
      radius = r;
    }

}

