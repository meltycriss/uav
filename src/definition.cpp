#include "definition.h"
using namespace std;

namespace uav{
  
  class Formation{
    public:
      Formation(){}

      Formation(
          vector<Polytope> _uavs,
          Polytope _convexHull,
          double _minDis,
          double _pref
          ):
        uavs(_uavs),
        convexHull(_convexHull),
        minDis(_minDis),
        pref(_pref){

        }

      vector<Polytope> uavs;
      Polytope convexHull;
      double minDis;
      double pref;
  };
}
