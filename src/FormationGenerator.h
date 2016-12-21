#ifndef FORMATION_GENERATOR_H
#define FORMATION_GENERATOR_H

namespace uav{
  class FormationGenerator{
    public:
      FormationGenerator(){}

      FormationGenerator(
          //lcp
          Point _gDir,
          std::vector<Polytope> _uavs,
          std::vector<Polytope> _uavShapes,
          std::vector<Polytope> _staticObstacles,
          std::vector<Polytope> _dynamicObstacles,
          double _timeInterval,
          double _currTime,
          std::vector<trajectory> _dynamicObstaclesTrajectories,
          //of
          std::vector<Formation> _formations,
          const Eigen::MatrixXd &_A,
          const Eigen::VectorXd &_b,
          const Point &_gPref,
          double _sPref,
          Eigen::Vector4d _qPref,
          double _wT,
          double _wS,
          double _wQ
          ):
        //lcp
        gDir_(_gDir),
        uavs_(_uavs),
        uavShapes_(_uavShapes),
        staticObstacles_(_staticObstacles),
        dynamicObstacles_(_dynamicObstacles),
        timeInterval_(_timeInterval),
        currTime_(_currTime),
        dynamicObstaclesTrajectories_(_dynamicObstaclesTrajectories),
        //of
        formations_(_formations),
        sA_(_A),
        sB_(_b),
        sG_(_gPref),
        sS_(_sPref),
        sQ_(_qPref),
        sWT_(_wT),
        sWS_(_wS),
        sWQ_(_wQ)
    {}


    private:
      //lcp
      Point gDir_;  //goal
      std::vector<Polytope> uavs_;
      std::vector<Polytope> uavShapes_;
      std::vector<Polytope> staticObstacles_;
      std::vector<Polytope> dynamicObstacles_; //relative coord w.r.t uav centroid
      double timeInterval_;
      double currTime_;
      std::vector<trajectory> dynamicObstaclesTrajectories_; //relative velocity
      //of
      std::vector<Formation> formations_;
      Eigen::MatrixXd sA_;
      Eigen::VectorXd sB_;
      Point sG_;
      double sS_;
      Eigen::Vector4d sQ_;
      double sWT_, sWS_, sWQ_;





  };

}


#endif
