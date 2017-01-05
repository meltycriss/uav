#ifndef LARGEST_CONVEX_POLYTOPE_H
#define LARGEST_CONVEX_POLYTOPE_H

#include <vector>
#include <Eigen/Core>
#include "iris/iris.h"
#include "iris/iris_mosek.h"
#include "definition.h"
#include "util.h"

namespace uav{

  class LargestConvexPolytope{
    public:

      //constructor
      LargestConvexPolytope(){}

      LargestConvexPolytope(
          const Point &_gDir,
          const std::vector<Polytope> &_uavs,
          const std::vector<Polytope> &_uavShapes, //for dilated volume
          const std::vector<Polytope> &_staticObstacles,
          const std::vector<Polytope> &_dynamicObstacles,
          const double &_timeInterval,
          const double &_currTime,
          const std::vector<trajectory> &_dynamicObstaclesTrajectories
          ):
        gDir_(_gDir),
        uavs_(_uavs),
        uavShapes_(_uavShapes),
        staticObstacles_(_staticObstacles),
        dynamicObstacles_(_dynamicObstacles),
        timeInterval_(_timeInterval),
        currTime_(_currTime),
        dynamicObstaclesTrajectories_(_dynamicObstaclesTrajectories){

        }

      //core function
      //return true if valid convex polytope, false otherwise
      //linear constraints is passed through _A and _B
      bool getLargestConvexPolytope(Eigen::MatrixXd&, Eigen::VectorXd&); //return 0 if status ii/iii

      //getter
      Point getGDir(){
        return gDir_;
      }
      std::vector<Polytope> getUavs(){
        return uavs_;
      }
      std::vector<Polytope> getUavShapes(){
        return uavShapes_;
      }
      std::vector<Polytope> getStaticObstacles(){
        return staticObstacles_;
      }
      std::vector<Polytope> getDynamicObstacles(){
        return dynamicObstacles_;
      }
      double getTimeInterval(){
        return timeInterval_;
      }
      double getCurrTime(){
        return currTime_;
      }
      std::vector<trajectory> getDynamicObstaclesTrajectories(){
        return dynamicObstaclesTrajectories_;
      }

      //setter
      void setGDir(const Point &_gDir){
        gDir_ = _gDir;
      }
      void setUavs(const std::vector<Polytope> &_uavs){
        uavs_ = _uavs;
      }
      void setUavShapes(const std::vector<Polytope> &_uavShapes){
        uavShapes_ = _uavShapes;
      }
      void setStaticObstacles(const std::vector<Polytope> &_staticObstacles){
        staticObstacles_ = _staticObstacles;
      }
      void setDynamicObstacles(const std::vector<Polytope> &_dynamicObstacles){
        dynamicObstacles_ = _dynamicObstacles;
      }
      void setTimeInterval(const double _timeInterval){
        timeInterval_ = _timeInterval;
      }
      void setCurrTime(const double _currTime){
        currTime_ = _currTime;
      }
      void setDynamicObstaclesTrajectories(const std::vector<trajectory> &_dynamicObstaclesTrajectories){
        dynamicObstaclesTrajectories_ = _dynamicObstaclesTrajectories;
      }


    private:
      //all coordinates are w.r.t formation centroid
      Point gDir_;  //goal
      std::vector<Polytope> uavs_;
      std::vector<Polytope> uavShapes_;
      std::vector<Polytope> staticObstacles_;
      std::vector<Polytope> dynamicObstacles_; //relative coord w.r.t uav centroid
      double timeInterval_;
      double currTime_;
      std::vector<trajectory> dynamicObstaclesTrajectories_; //relative velocity

    private:
    //public:
      //return (O+V)x[0,TimeInterval] U Union(U(D+V)xt)
      std::vector<Polytope4d> getObstacleBar();

      //return true if valid convex polytope, false otherwise
      //region of directed largest convex region in free space is return through param
      bool directedLargestConvexRegionInFreeSpace(iris::IRISRegion &region, const std::vector<Point4d> &in, const Point4d &dir, const std::vector<Polytope4d> &obstacles) const;

  };


}

#endif
