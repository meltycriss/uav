#include <Eigen/Core>
#include <vector>
#include "util.h"
#include "definition.h"
#include "OptimalFormation.h"
#include "snoptProblem.hpp"

#include "iris/iris.h"

using namespace std;

namespace uav{

  // initialze static variables
  bool OptimalFormation::sInitialized_ = false;
  Formation OptimalFormation::sFormation_ = Formation();
  Point OptimalFormation::sG_ = Point();
  double OptimalFormation::sS_ = 1;
  Eigen::Vector3d OptimalFormation::sQ_ = Eigen::Vector3d();
  double OptimalFormation::sWT_ = 1;
  double OptimalFormation::sWS_ = 1;
  double OptimalFormation::sWQ_ = 1;
  Eigen::MatrixXd OptimalFormation::sA_ = Eigen::MatrixXd();
  Eigen::VectorXd OptimalFormation::sB_ = Eigen::VectorXd();

  void OptimalFormation::sUsrfg_( int    *Status, int *n,    double x[],
      int    *needF,  int *neF,  double F[],
      int    *needG,  int *neG,  double G[],
      char      *cu,  int *lencu,
      int    iu[],    int *leniu,
      double ru[],    int *lenru ){
    //interact with iris
    int rowsA = sA_.rows();
    int colsA = sA_.cols();
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
        F[i] = sA_(i-1, 0) * x[0] + sA_(i-1, 1) * x[1] + sA_(i-1, 2) * x[2];
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
          G[i*3+j] = sA_(i-1,j);
        }
      }
    }
  }

  // return index of optimal formation
  // deviation is passed through param
  Vector8d OptimalFormation::optimalDeviation(const Formation &formation){
    // static variables required initializing
    if(!sInitialized_){
      throw OptimalFormationError();
    }

    //SNOPT
    int rowsA = sA_.rows();
    int colsA = sA_.cols();
    Polytope fs = sFormation_.convexHull;
    double radius = sFormation_.radius;
    double minDis = sFormation_.minInterDis;

    snoptProblemA ToyProb;

    // Allocate and initialize;
    // t0 t1 t2 s q0 q1 q2 q3
    int n     =  8; 
    // first term : obj_func
    // second term: each vertice satisfies Az<=b, 
    // third term : C2,C3
    int neF   =  1 + rowsA * fs.size() + 2;

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
      xlow[i] = -INFI;
      xupp[i] = INFI;
      xstate[i] = 0;
    }

    // set the upper and lower bounds of F
    // [0]         : obj
    // [1-#v*#rows]: C1
    // [#v*#rows+1]: C2
    // [#v*#rows+2]: C3
    Flow[0] = -INFI;
    Fupp[0] = INFI;
    for(int i=0; i<fs.size(); ++i){
      for(int j=0; j<rowsA; ++j){
        Flow[i*rowsA+j+1] = -INFI;
        Fupp[i*rowsA+j+1] = sB_(j);
        Fmul[i*rowsA+j+1] = 0;
      }
    }
    Flow[fs.size()*rowsA+1] = -INFI;
    Fupp[fs.size()*rowsA+1] = (-2.0) * radius / (minInterDis);
    Flow[fs.size()*rowsA+2] = 1.0;
    Fupp[fs.size()*rowsA+2] = 1.0;

    // set the indicator of G
    for(int i=0; i<neF; ++i){
      for(int j=0; j<n; ++j){
        iGfun[i*n+j] = i;
        jGvar[i*n+j] = j;
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
    ToyProb.setUserFun     ( sUsrfg_ );      // Sets the usrfun that supplies G and F.
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


    return Vector8d();
  }

}
