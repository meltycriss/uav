#include <iostream>
#include <Eigen/Core>
#include "iris/iris.h"
#include <cmath>
#include <limits.h>

#include <stdio.h>
#include <string.h>
#include <iostream>

#include "sntoyA.hpp"
#include "snoptProblem.hpp"

using namespace std;

//constraints from IRIS
Eigen::MatrixXd A;
Eigen::VectorXd B;

//const double INFI = 1e20;
const double INFI = INT_MAX;
const double NEG_INFI = -INFI;

void toyusrfg_( int    *Status, int *n,    double x[],
    int    *needF,  int *neF,  double F[],
    int    *needG,  int *neG,  double G[],
    char      *cu,  int *lencu,
    int    iu[],    int *leniu,
    double ru[],    int *lenru )
{
  //interact with iris
  int rowsA = A.rows();
  int colsA = A.cols();
  if(colsA != 3){
    cout << "[error] colsA != 3" << endl;
    return;
  }

  if ( *needF > 0 ) {
    //objective func: dis to point(2sqrt(3),2,3)
    F[0] = (x[0] - 2*sqrt(3)) * (x[0] - 2*sqrt(3))
      + (x[1] - 2) * (x[1] - 2)
      + (x[2] - 3) * (x[2] - 3);

    //constraints
    for(int i=1; i<=rowsA; ++i){
      F[i] = A(i-1, 0) * x[0] + A(i-1, 1) * x[1] + A(i-1, 2) * x[2];
    }
  }

  if ( *needG > 0 ) {
    //iGfun[i*3+j] = i
    //jGvar[i*3+j] = j

    //derivative of objcetive func
    G[0] = 2*(x[0]-2*sqrt(3));
    G[1] = 2*(x[1]-2);
    G[2] = 2*(x[2]-3);

    //derivative of constraints
    for(int i=1; i<=rowsA; ++i){
      for(int j=0; j<3; ++j){
        G[i*3+j] = A(i-1,j);
      }
    }
  }
}

int main(int argc, char** argv) {
  //------------------------------IRIS part------------------------------

  //problem:
  //    obstacle_pts
  //    bounds
  //    dim
  //    seed
  iris::IRISProblem problem(3);
  problem.setSeedPoint(Eigen::Vector3d(0.1, 0.1, 0.1));

  // format the output so as to visualize in jupyter conveniently
  Eigen::IOFormat np_array(Eigen::StreamPrecision, 0, ", ", ",\n", "[", "]", "np.array([", "])");

  // obs: dim * num of points
  Eigen::MatrixXd obs(3,4);

  obs << sqrt(3),     0,     0,    sqrt(3),
      0,     0,     0,          0,
      -INFI, -INFI,  INFI,       INFI;
  problem.addObstacle(obs);
  //std::cout << "obs1 = " << obs.format(np_array) << std::endl;

  obs << sqrt(3),     0,     0,    sqrt(3),
      1,     1,     1,          1,
      -INFI, -INFI,  INFI,       INFI;
  problem.addObstacle(obs);
  obs <<       0,     0,     0,          0,
      0,     1,     1,          0,
      -INFI, -INFI,  INFI,       INFI;
  problem.addObstacle(obs);
  obs << sqrt(3), sqrt(3), sqrt(3), sqrt(3),
      0,       1,       1,       0,
      -INFI,   -INFI,    INFI,    INFI;
  problem.addObstacle(obs);
  obs << sqrt(3), sqrt(3), 0, 0,
      0,       1, 1, 0,
      0,       0, 0, 0;
  problem.addObstacle(obs);
  obs << sqrt(3), sqrt(3),     0,     0,
      0,       1,     1,     0,
      3.0/2,   3.0/2, 3.0/2, 3.0/2;
  problem.addObstacle(obs);

  // option require_containment
  iris::IRISOptions options;
  iris::IRISRegion region = inflate_region(problem, options);


  //std::cout << "a = " << region.polyhedron.getA().format(np_array) << std::endl;
  //std::cout << "b = " << region.polyhedron.getB().format(np_array) << std::endl;

  //std::cout << "c = " << region.ellipsoid.getC().format(np_array) << std::endl;
  //std::cout << "d = " << region.ellipsoid.getD().format(np_array) << std::endl;

  A = region.polyhedron.getA();
  B = region.polyhedron.getB();

  //cout << "A:" << endl << A << endl
  //  << "B:" << endl << B << endl;


  //------------------------------SNOPT part------------------------------

  int rowsA = A.rows();
  int colsA = A.cols();

  snoptProblemA ToyProb;

  // Allocate and initialize;
  int n     =  colsA;
  int neF   =  rowsA+1; // #constrains + obj_func

  double *x      = new double[n];
  double *xlow   = new double[n];
  double *xupp   = new double[n];
  double *xmul   = new double[n];
  int    *xstate = new    int[n];

  double *F      = new double[neF];
  double *Flow   = new double[neF];
  double *Fupp   = new double[neF];
  double *Fmul   = new double[neF];
  int    *Fstate = new int[neF];

  int    ObjRow  = 0;
  double ObjAdd  = 0;

  int Cold = 0, Basis = 1, Warm = 2;

  int lenG   = n * neF;
  int *iGfun = new int[lenG];
  int *jGvar = new int[lenG];
  int neG;

  // set the upper and lower bounds of x
  for(int i=0; i<n; ++i){
    x[i] = 1;
    xlow[i] = NEG_INFI;
    xupp[i] = INFI;
    xstate[i] = 0;
  }

  // set the upper and lower bounds of F
  Flow[0] = NEG_INFI;
  Fupp[0] = INFI;
  for(int i=1; i<neF; ++i){
    Flow[i] = NEG_INFI;
    Fupp[i] = B(i-1);
    Fmul[i] = 0;
  }

  // set the indicator of G
  for(int i=0; i<neF; ++i){
    for(int j=0; j<3; ++j){
      iGfun[i*3+j] = i;
      jGvar[i*3+j] = j;
    }
  }
  neG = lenG;

  // Load the data for ToyProb ...
  ToyProb.setProbName    ("uav");
  ToyProb.setPrintFile   ( "uav.out" );
  ToyProb.setProblemSize ( n, neF );
  ToyProb.setObjective   ( ObjRow, ObjAdd );
  ToyProb.setX           ( x, xlow, xupp, xmul, xstate );
  ToyProb.setF           ( F, Flow, Fupp, Fmul, Fstate );
  ToyProb.setG           ( lenG, neG, iGfun, jGvar );
  ToyProb.setUserFun     ( toyusrfg_ );      // Sets the usrfun that supplies G and F.
  //ToyProb.setSpecsFile   ( "sntoya.spc" );
  ToyProb.setIntParameter( "Derivative option", 1 );
  ToyProb.setIntParameter( "Major Iteration limit", 250 );
  ToyProb.setIntParameter( "Verify level ", 3 );
  ToyProb.solve          ( Cold );

  for (int i = 0; i < n; i++ ){
    cout << "x = " << x[i] << " xstate = " << xstate[i] << endl;
  }
  for (int i = 0; i < neF; i++ ){
    cout << "F = " << F[i] << " Fstate = " << Fstate[i] << endl;
  }

  delete []iGfun;  delete []jGvar;

  delete []x;      delete []xlow;   delete []xupp;
  delete []xmul;   delete []xstate;

  delete []F;      delete []Flow;   delete []Fupp;
  delete []Fmul;   delete []Fstate;


  return 0;
}
