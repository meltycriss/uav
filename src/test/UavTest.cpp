#include "LargestConvexPolytope.h"
#include "OptimalFormation.h"
#include <iostream>
#include <string>
#include "drake/math/quaternion.h"
#include "util.h"
#include "iris/geometry.h"
#include <cmath>
#include <cstdlib>
#include <vector>
#include <RVO.h>
#include "scene.pb.h"
#include <fstream>
using namespace std;
using namespace uav;

//visualization helper
scene::Scene scn;
scene::Uavs *scnUavs;
scene::UavsDir *scnUavsDir;
scene::StaticObstacles *scnSos;
scene::DynamicObstacles *scnDos;
scene::GDir *scnGDir;
scene::A *scnA;
scene::B *scnB;

#ifndef M_PI
const float M_PI = 3.14159265358979323846f;
#endif

const double DELTA = 1.0;

//for display
Eigen::IOFormat np_array(Eigen::StreamPrecision, 0, ", ", ",\n", "[", "]", "np.array([", "])");

//current uavs centroid
Point currCentroid = Point(0,0,0);

void printScene(ostream &stm){
  string str;
  scn.SerializeToString(&str);
  stm << str << endl << "###" << endl;

//  stm << "gDir: " << scn.gdir().x() << " " << scn.gdir().y() << endl;
//  stm << "A: " << scn.a().row() << " " << scn.a().col() << endl;
//  stm << "scnA: " << scnA.row() << " " << scnA.col() << endl;

}


Point traj1(Point p, double t){
  Point res = p;
  Point dir(0, -1, 0);
  int auxT = floor(t);
  t = t - (auxT/18)*18;
  t = t<9 ? t:18-t;
  res += dir * t;
  //relative coordinate
  res -= currCentroid;
  return res;
}

// \return  formationIdx:    if CA formation, formation pos is set
//                    -1:    if no CA formation, all uavs' goal are set to gDir
// \caution both input and output are in relative coordinates
int getFormationGoal(
    //result
    vector<Point> &rFormationGoal,
    //lcp
    const Point &_gDir,
    const std::vector<Polytope> &_uavs,
    const std::vector<Polytope> &_uavShapes,
    const std::vector<Polytope> &_staticObstacles,
    const std::vector<Polytope> &_dynamicObstacles,
    const double &_timeInterval,
    const double &_currTime,
    const std::vector<trajectory> &_dynamicObstaclesTrajectories,
    //of
    const std::vector<Formation> &_formations,
    const double &_sPref,
    const Eigen::Vector4d &_qPref,
    const double &_wT,
    const double &_wS,
    const double &_wQ
    ){

  //----------------------------------------------------------------------------
  //  solving the problem under relative coordinates
  //----------------------------------------------------------------------------

  //get lcp
  LargestConvexPolytope lcp(
      _gDir,
      _uavs,
      _uavShapes,
      _staticObstacles,
      _dynamicObstacles,
      _timeInterval,
      _currTime,
      _dynamicObstaclesTrajectories);

  Eigen::MatrixXd lcpA;
  Eigen::VectorXd lcpB;
  bool shouldFormation = lcp.getLargestConvexPolytope(lcpA, lcpB);

  //----------------------------------------------------------------------------
  //  assign goal for each uav
  //----------------------------------------------------------------------------

  //for visualize

  //CA formation exists
  if(shouldFormation){
    //largest convex polytope debug info
    Eigen::MatrixXd disp_A = lcpA;
    Eigen::VectorXd disp_B = lcpB;
    reducePolyDim(disp_A, disp_B, 2, 0);
    reducePolyDim(disp_A, disp_B, 2, _timeInterval);

    //for visualize
    scnA = new scene::A();
    scnA->set_row(disp_A.rows());
    scnA->set_col(disp_A.cols());
    for(int i=0; i<disp_A.rows(); ++i){
      for(int j=0; j<disp_A.cols(); ++j){
        scnA->add_data(disp_A(i,j));
      }
    }
    scn.set_allocated_a(scnA);

    scnB = new scene::B();
    scnB->set_row(disp_B.rows());
    for(int i=0; i<disp_B.rows(); ++i){
      scnB->add_data(disp_B(i));
    }
    scn.set_allocated_b(scnB);

    cout << "a = " << lcpA.format(np_array) << endl;
    cout << "b = " << lcpB.format(np_array) << endl;
    cout << "a = " << disp_A.format(np_array) << endl;
    cout << "b = " << disp_B.format(np_array) << endl; cout << "INFI = " << INFI << endl;
    //
    cout << "-----------------------------------------------------------" << endl;
    cout << "gDir: " << endl << _gDir + currCentroid << endl;

    cout << "-----------------------------------------------------------" << endl;
    cout << "centroidBefore:" << endl << currCentroid << endl;
    cout << "-----------------------------------------------------------" << endl;
    for(int i=0; i<_uavs.size(); ++i){
      Polytope poly = _uavs[i];
      cout << "uavBefore" << i << endl;
      for(int j=0; j<poly.size(); ++j){
        cout << poly[j] + currCentroid << endl;
      }
    }

    //optimal deviation
    OptimalFormation of(_formations);
    OptimalFormation::init(lcpA, lcpB, _gDir, _sPref, _qPref, _wT, _wS, _wQ, _timeInterval);
    Vector8d optimalParam;
    int index = of.optimalFormation(optimalParam);


    vector<Polytope> formationUavs = tsqTransPolyVec(_formations[index].uavs, optimalParam);

    rFormationGoal.resize(formationUavs.size());
    for(int i=0; i<rFormationGoal.size(); ++i){
      rFormationGoal[i] = getCentroid(formationUavs[i]);
    }
    return index;

  }
  //no CA formation
  else{
    //assign goal for each uav (local planner)
    rFormationGoal.resize(_uavs.size());
    for(int i=0; i<rFormationGoal.size(); ++i){
      rFormationGoal[i] = _gDir;
    }
    return -1;
  }
}

/* Store the goals of the agents. */
std::vector<RVO::Vector2> goals;

void setupScenario(RVO::RVOSimulator *sim,
    const double &_timeStep,
    const vector<RVO::Vector2> &_uavs,
    const vector<RVO::Vector2> &_goals,
    const vector<vector<RVO::Vector2> > &_obstacles,
    float _neighborDist, size_t _maxNeighbors,
    float _timeHorizon, float _timeHorizonObst,
    float _radius, float _maxSpeed,
    const RVO::Vector2 &_velocity = RVO::Vector2()
    ){
  /* Specify the global time step of the simulation. */
  sim->setTimeStep(_timeStep);

  //void setAgentDefaults(float neighborDist, size_t maxNeighbors,
  //                float timeHorizon, float timeHorizonObst,
  //                float radius, float maxSpeed,
  //                const Vector2 &velocity = Vector2());

  /* Specify the default parameters for agents that are subsequently added. */
  //sim->setAgentDefaults(5.0f, 10, 5.0f, 5.0f, 1.0f, 2.0f);
  sim->setAgentDefaults(_neighborDist, _maxNeighbors, _timeHorizon, _timeHorizonObst, _radius, _maxSpeed);

  /*
   * Add agents, specifying their start position, and store their goals on the
   * opposite side of the environment.
   */
  goals.resize(_uavs.size());
  for(int i=0; i<_uavs.size(); ++i){
    sim->addAgent(_uavs[i]);
    goals[i] = _goals[i];
  }


  /*
   * Add (polygonal) obstacles, specifying their vertices in counterclockwise
   * order.
   */
  for(int i=0; i<_obstacles.size(); ++i){
    vector<RVO::Vector2> obs = _obstacles[i];
    sim->addObstacle(obs);
  }

  /* Process the obstacles so that they are accounted for in the simulation. */
  sim->processObstacles();
}

void setPreferredVelocities(RVO::RVOSimulator *sim)
{
  /*
   * Set the preferred velocity to be a vector of unit magnitude (speed) in the
   * direction of the goal.
   */
  for (int i = 0; i < static_cast<int>(sim->getNumAgents()); ++i) {
    RVO::Vector2 goalVector = goals[i] - sim->getAgentPosition(i);

    if (RVO::absSq(goalVector) > 1.0f) {
      goalVector = RVO::normalize(goalVector);
    }

    sim->setAgentPrefVelocity(i, goalVector);

    /*
     * Perturb a little to avoid deadlocks due to perfect symmetry.
     */
    float angle = std::rand() * 2.0f * M_PI / RAND_MAX;
    float dist = std::rand() * 0.0001f / RAND_MAX;

    sim->setAgentPrefVelocity(i, sim->getAgentPrefVelocity(i) +
        dist * RVO::Vector2(std::cos(angle), std::sin(angle)));
  }
}

bool reachedGoal(RVO::RVOSimulator *sim)
{
  /* Check if all agents have reached their goals. */
  for (size_t i = 0; i < sim->getNumAgents(); ++i) {
    if (RVO::absSq(sim->getAgentPosition(i) - goals[i]) > DELTA) {
      return false;
    }
  }
  return true;
}

void updateVisualization(RVO::RVOSimulator *sim)
{
  /* Output the current global time. */
  std::cout << sim->getGlobalTime();

  /* Output the current position of all the agents. */
  for (size_t i = 0; i < sim->getNumAgents(); ++i) {
    std::cout << " " << sim->getAgentPosition(i);
  }

  std::cout << std::endl;
}



int main(){

  fstream fd("scenes.txt", ios::out);

  //----------------------------------------------------------------------------
  //  initialization of scenerio under absolute coordinates
  //----------------------------------------------------------------------------

  //aux var
  Point p;

  //gDir
  Point gDir;
  gDir << 14,10,0;
  gDir << 3.5,10,0;
  gDir << 0,10,0;

  //uavs
  vector<Polytope> uavs;
  Polytope uav;
  //uav0
  p << -2,2,0;
  uav.clear();
  uav.push_back(p);
  uavs.push_back(uav);
  //uav1
  p << 2,2,0;
  uav.clear();
  uav.push_back(p);
  uavs.push_back(uav);
  //uav2
  p << 2,-2,0;
  uav.clear();
  uav.push_back(p);
  uavs.push_back(uav);
  //uav3
  p << -2,-2,0;
  uav.clear();
  uav.push_back(p);
  uavs.push_back(uav);

  //uavShapes
  vector<Polytope> uavShapes;
  Polytope uavShape;
  //us0
  p << -1,1,0;
  uavShape.push_back(p);
  p << -1,-1,0;
  uavShape.push_back(p);
  p << 1,-1,0;
  uavShape.push_back(p);
  p << 1,1,0;
  uavShape.push_back(p);
  uavShapes.push_back(uavShape);

  //convex hull of formation
  Polytope convexHull;
  p << -3,3,0;
  convexHull.push_back(p);
  p << -3,-3,0;
  convexHull.push_back(p);
  p << 3,-3,0;
  convexHull.push_back(p);
  p << 3,3,0;
  convexHull.push_back(p);

  //template formation
  //caution: uavs in formation is not necessarily the same as actual uavs
  vector<Formation> formations;
  Formation formation(uavs, uavShapes, convexHull, 1);
  formations.push_back(formation);

  //staticObstacles
  vector<Polytope> staticObstacles;
  Polytope staticObstacle;
  //so0
  p << 5,4,0;
  staticObstacle.push_back(p);
  p << 5,1,0;
  staticObstacle.push_back(p);
  p << 8,1,0;
  staticObstacle.push_back(p);
  p << 8,4,0;
  staticObstacle.push_back(p);
  staticObstacles.push_back(staticObstacle);

  //dynamicObstacles and corresponding trajectory function
  vector<Polytope> dynamicObstacles;
  vector<trajectory> dynamicObstaclesTrajectories;
  //do0
  Polytope dynamicObstacle;
  trajectory dynamicObstaclesTrajectory;
  p << 10,10,0;
  dynamicObstacle.push_back(p);
  p << 10,9,0;
  dynamicObstacle.push_back(p);
  p << 11,9,0;
  dynamicObstacle.push_back(p);
  p << 11,10,0;
  dynamicObstacle.push_back(p);
  dynamicObstaclesTrajectory = &traj1;
  dynamicObstacles.push_back(dynamicObstacle);
  dynamicObstaclesTrajectories.push_back(dynamicObstaclesTrajectory);

  //timeInterval
  double timeInterval = 1;

  //currTime
  double currTime = 0;

  //weight of optimization cost
  double wT = 1;
  double wS = 1;
  double wQ = 1;

  //prefered param
  double sPref = 1;
  Eigen::Vector4d qPref = Eigen::Vector4d(1,0,0,0);

  //curr goal for each uavs
  vector<Point> uavsDir(uavs.size());

  int counter = 0;

  //looping
  while(counter<2){
    //----------------------------------------------------------------------------
    //  translating absolute coordinates to relative coordinates
    //----------------------------------------------------------------------------

    Point gDirRela = gDir - currCentroid;
    vector<Polytope> uavsRela = absToRela(uavs, currCentroid);
    vector<Polytope> staticObstaclesRela = absToRela(staticObstacles, currCentroid);
    //relative coordinates of dynamicObstacles is handled by its trajectory funciton

    //----------------------------------------------------------------------------
    //  solving the problem under relative coordinates
    //----------------------------------------------------------------------------

    vector<Point> formationGoalRela;
    int formationIdx = getFormationGoal(
        //result
        formationGoalRela,
        //lcp
        gDirRela,
        uavsRela,
        uavShapes,
        staticObstaclesRela,
        dynamicObstacles,
        timeInterval,
        currTime,
        dynamicObstaclesTrajectories,
        //of
        formations,
        sPref,
        qPref,
        wT,
        wS,
        wQ
        );

    //----------------------------------------------------------------------------
    //  translating relative coordinates to absolute coordinates
    //----------------------------------------------------------------------------

    //formation goal in absolute coordinates
    vector<Point> formationGoal = relaToAbs(formationGoalRela, currCentroid);

    //----------------------------------------------------------------------------
    //  matching and assigning
    //----------------------------------------------------------------------------

    //matching with respect to distance
    Eigen::MatrixXd matCost = getDisMat(uavs, formationGoal);
    vector<int> vecAssign = hungarian(matCost);
    //assign goal for each uav (formation)
    for(int i=0; i<vecAssign.size(); ++i){
      int assignedIdx = vecAssign[i];
      uavsDir[i] = formationGoal[assignedIdx];
    }

    //----------------------------------------------------------------------------
    //  navigate to goal with ORCA
    //----------------------------------------------------------------------------


    //----------------------------------------------------------------------------
    //  adapt to RVO data structure
    //----------------------------------------------------------------------------

    //uavs
    vector<RVO::Vector2> uavsRVO;
    for(int i=0; i<uavs.size(); ++i){
      Point uavCentroid = getCentroid(uavs[i]);
      RVO::Vector2 uavRVO(uavCentroid(0), uavCentroid(1));
      uavsRVO.push_back(uavRVO);
    }
    //uavsDir
    vector<RVO::Vector2> goalsRVO;
    for(int i=0; i<uavsDir.size(); ++i){
      Point uavDir = uavsDir[i];
      RVO::Vector2 goalRVO(uavDir(0), uavDir(1));
      goalsRVO.push_back(goalRVO);
    }
    //obstacles
    vector<vector<RVO::Vector2> > obstaclesRVO;
    //so
    for(int i=0; i<staticObstacles.size(); ++i){
      Polytope obs = staticObstacles[i];
      vector<RVO::Vector2> obsRVO;
      for(int j=0; j<obs.size(); ++j){
        Point p = obs[j];
        RVO::Vector2 pRVO(p(0), p(1));
        obsRVO.push_back(pRVO);
      }
      obstaclesRVO.push_back(obsRVO);
    }
    //do
    for(int i=0; i<dynamicObstacles.size(); ++i){
      Polytope obs = dynamicObstacles[i];
      vector<RVO::Vector2> obsRVO;
      for(int j=0; j<obs.size(); ++j){
        Point p = obs[j];
        p = dynamicObstaclesTrajectories[i](p, currTime+timeInterval) + currCentroid; //rela to abs
        RVO::Vector2 pRVO(p(0), p(1));
        obsRVO.push_back(pRVO);
      }
      obstaclesRVO.push_back(obsRVO);
    }

    //----------------------------------------------------------------------------
    //  hyper-param for ORCA
    //----------------------------------------------------------------------------

    double timeStep = 0.25;
    double radiusRVO = getRadius(uavShapes[0]);
    double neighborDistRVO = 5 * radiusRVO;
    int maxNeighborsRVO = uavs.size();
    double timeHorizonRVO = 5;
    double timeHorizonObstRVO = 5;
    double maxSpeedRVO = 2;

    //----------------------------------------------------------------------------
    //  ORCA
    //----------------------------------------------------------------------------

    /* Create a new simulator instance. */
    RVO::RVOSimulator *sim = new RVO::RVOSimulator();

    // to do: set param
    /* Set up the scenario. */
    setupScenario(sim, timeStep, uavsRVO, goalsRVO, obstaclesRVO,
        neighborDistRVO, maxNeighborsRVO, timeHorizonRVO,
        timeHorizonObstRVO, radiusRVO, maxSpeedRVO);

    //int count = 0;
    /* Perform (and manipulate) the simulation. */
    do {
      // if(count%5==0){
      //   updateVisualization(sim);
      // }
      setPreferredVelocities(sim);
      sim->doStep();
      //++count;
    }
    while (!reachedGoal(sim));


    //----------------------------------------------------------------------------
    //  update scene::Scene scn for visualization
    //----------------------------------------------------------------------------
    //
    //
    //

    scnUavs = new scene::Uavs();
    scene::Uav *scnUav;
    for(int i=0; i<uavsRVO.size(); ++i){
      scnUav = scnUavs->add_uav();
      RVO::Vector2 pRVO = uavsRVO[i];
      scnUav->set_x(pRVO.x());
      scnUav->set_y(pRVO.y());
    }
    scn.set_allocated_uavs(scnUavs);

    scnUavsDir = new scene::UavsDir();
    scene::Point *scnPoint;
    for(int i=0; i<goalsRVO.size(); ++i){
      scnPoint = scnUavsDir->add_uavdir();
      RVO::Vector2 pRVO = goalsRVO[i];
      scnPoint->set_x(pRVO.x());
      scnPoint->set_y(pRVO.y());
    }
    scn.set_allocated_uavsdir(scnUavsDir);

    //order-sensitive
    scnSos = new scene::StaticObstacles();
    scene::Polytope *scnPoly;
    for(int i=0; i<staticObstacles.size(); ++i){
      vector<RVO::Vector2> polyRVO = obstaclesRVO[i];
      scnPoly = scnSos->add_so();
      for(int j=0; j<polyRVO.size(); ++j){
        RVO::Vector2 pRVO = polyRVO[j];
        scnPoint = scnPoly->add_point();
        scnPoint->set_x(pRVO.x());
        scnPoint->set_y(pRVO.y());
      }
    }
    scn.set_allocated_sos(scnSos);

    scnDos = new scene::DynamicObstacles();
    for(int i=staticObstacles.size(); i<obstaclesRVO.size(); ++i){
      vector<RVO::Vector2> polyRVO = obstaclesRVO[i];
      scnPoly = scnDos->add_do_();
      for(int j=0; j<polyRVO.size(); ++j){
        RVO::Vector2 pRVO = polyRVO[j];
        scnPoint = scnPoly->add_point();
        scnPoint->set_x(pRVO.x());
        scnPoint->set_y(pRVO.y());
      }
    }
    scn.set_allocated_dos(scnDos);

    scnGDir = new scene::GDir();
    scnGDir->set_x(gDir(0));
    scnGDir->set_y(gDir(1));
    scn.set_allocated_gdir(scnGDir);
    //scnA and scnB are set in function
    
    cout << "-----------------------------------------------------------" << endl;
    cout << "testing Protobuf" << endl;

    printScene(fd);
    printScene(cout);

    scn.Clear();

    //----------------------------------------------------------------------------
    //  update pos info
    //----------------------------------------------------------------------------

    //update uav position
    for(int i=0; i<uavsDir.size(); ++i){
      RVO::Vector2 pRVO = sim->getAgentPosition(i);
      Point p(pRVO.x(), pRVO.y(), 0);
      uavs[i] = uavMoveTo(p, uavs[i]);
    }

    //update currCentroid
    currCentroid = Point(0,0,0);
    for(int i=0; i<uavs.size(); ++i){
      currCentroid += getCentroid(uavs[i]);
    }
    currCentroid /= uavs.size();

    //timer
    currTime += sim->getGlobalTime();

    cout << "-----------------------------------------------------------" << endl;
    cout << "centroidAfter:" << endl;
    cout << "goal" << endl << getCentroid(uavsDir) << endl;
    cout << "actual" << endl << currCentroid << endl;
    cout << "-----------------------------------------------------------" << endl;
    for(int i=0; i<uavs.size(); ++i){
      Polytope poly = uavs[i];
      cout << "uavAfter" << i << endl;
      cout << "goal" << endl;
      cout << uavsDir[i] << endl;
      cout << "actual" << endl;
      for(int j=0; j<poly.size(); ++j){
        cout << poly[j] << endl;
      }
    }



    //delete simulator instance
    delete sim;

    //update gDir
    gDir << 3,10,0;

    ++counter;
  }

  fd.close();

  return 0;
}

