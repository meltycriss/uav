#ifndef LARGEST_CONVEX_POLYTOPE_H
#define LARGEST_CONVEX_POLYTOPE_H

#include <vector>
#include <Eigen/Core>
#include "iris/iris.h"
#include "iris/iris_mosek.h"

class LargestConvexPolytope{
  public:
    //definition
    typedef Eigen::Vector3d Point;
    typedef Eigen::Vector4d Point4d;
    typedef std::vector<Point> Polytope;
    typedef Point (*trajectory)(Point, double);
    typedef std::vector<Point4d> Polytope4d;

    //constructor
    LargestConvexPolytope(){}

    LargestConvexPolytope(
        Point _gDir,
        std::vector<Polytope> _uavs,
        std::vector<Polytope> _uavShapes, //for dilated volume
        std::vector<Polytope> _staticObstacles,
        std::vector<Polytope> _dynamicObstacles,
        double _timeInterval,
        double _currTime,
        std::vector<trajectory> _dynamicObstaclesTrajectories
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
    void setGDir(Point _gDir){
      gDir_ = _gDir;
    }
    void setUavs(std::vector<Polytope> _uavs){
      uavs_ = _uavs;
    }
    void setUavShapes(std::vector<Polytope> _uavShapes){
      uavShapes_ = _uavShapes;
    }
    void setStaticObstacles(std::vector<Polytope> _staticObstacles){
      staticObstacles_ = _staticObstacles;
    }
    void setDynamicObstacles(std::vector<Polytope> _dynamicObstacles){
      dynamicObstacles_ = _dynamicObstacles;
    }
    void setTimeInterval(double _timeInterval){
      timeInterval_ = _timeInterval;
    }
    void setCurrTime(double _currTime){
      currTime_ = _currTime;
    }
    void setDynamicObstaclesTrajectories(std::vector<trajectory> _dynamicObstaclesTrajectories){
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
    //return (O+V)x[0,TimeInterval] U Union(U(D+V)xt)
    std::vector<Polytope4d> getObstacleBar();

    Polytope minkowskiSum(const Polytope &lhs, const Polytope &rhs) const;

    //return polytope after moving
    Polytope polytopeAfterMove(const Polytope &poly, double time, trajectory m) const;

    //return poly4d appending x to each vertice in poly3d
    Polytope4d poly3dToPoly4d(const Polytope &poly3d, double x) const;

    //return true if valid convex polytope, false otherwise
    //region of directed largest convex region in free space is return through param
    bool directedLargestConvexRegionInFreeSpace(iris::IRISRegion &region, const std::vector<Point4d> &in, const std::vector<Polytope4d> &obstacles) const;

    //return bounds generated from lb to ub
    iris::Polyhedron getPolyFromBounds(const Point4d&, const Point4d&) const;


};

#endif
