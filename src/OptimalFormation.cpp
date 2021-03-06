#include <Eigen/Core>
#include <vector>
#include "util.h"
#include "definition.h"
#include "OptimalFormation.h"
#include "snoptProblem.hpp"
#include "drake/math/quaternion.h"

#include "iris/iris.h"
#include <time.h>
#include <stdlib.h>

using namespace std;

namespace uav{

  // initialze static variables
  bool OptimalFormation::sInitialized_ = false;
  Formation OptimalFormation::sFormation_ = Formation();
  Point OptimalFormation::sG_ = Point();
  double OptimalFormation::sS_ = 1;
  Eigen::Vector4d OptimalFormation::sQ_ = Eigen::Vector4d();
  double OptimalFormation::sWT_ = 3;
  double OptimalFormation::sWS_ = 1;
  double OptimalFormation::sWQ_ = 1;
  double OptimalFormation::sTimeInterval_ = 0.46;
  Eigen::MatrixXd OptimalFormation::sA_ = Eigen::MatrixXd();
  Eigen::VectorXd OptimalFormation::sB_ = Eigen::VectorXd();

  void OptimalFormation::sUsrfg_( int    *Status, int *n,    double x[],
      int    *needF,  int *neF,  double F[],
      int    *needG,  int *neG,  double G[],
      char      *cu,  int *lencu,
      int    iu[],    int *leniu,
      double ru[],    int *lenru ){
    // x:
    // t0 t1 t2 s q0 q1 q2 q3
    // F:
    // [0]         : obj
    // [1-#v*#rows]: C1
    // [#v*#rows+1]: C2
    // [#v*#rows+2]: C3

    //interact with iris
    int rowsA = sA_.rows();
    int colsA = sA_.cols();
    Polytope fs = sFormation_.convexHull;
    double radius = sFormation_.radius;
    double minDis = sFormation_.minInterDis;
    double pref = sFormation_.pref;

    // t
    double xx0 = x[0] - sG_(0);
    double xx1 = x[1] - sG_(1);
    double xx2 = x[2] - sG_(2);
    // s
    double xx3 = x[3] - sS_;
    // q
    double xx4 = x[4] - sQ_(0);
    double xx5 = x[5] - sQ_(1);
    double xx6 = x[6] - sQ_(2);
    double xx7 = x[7] - sQ_(3);
    // t^2, s^2, q^2
    double ttSquare = xx0*xx0 + xx1*xx1 + xx2*xx2;
    double ssSquare = xx3*xx3;
    double qqSquare = xx4*xx4 + xx5*xx5 + xx6*xx6 + xx7*xx7;

    // functions
    if ( *needF > 0 ) {
      //obj function
      F[0] = sWT_ * ttSquare + sWS_ * ssSquare + sWQ_ * qqSquare + pref;

      //constraints
      //C1
      for(int i=0; i<fs.size(); ++i){
        if(rowsA>0){
          Eigen::Vector3d _t;
          double _s;
          Eigen::Vector4d _q;
          Eigen::Vector3d _f;
          _t << x[0], x[1], x[2];
          _s = x[3];
          _q << x[4], x[5], x[6], x[7];
          _f = fs[i];
          Eigen::Vector3d _x = _t + _s * drake::math::quatRotateVec(_q, _f);
          Eigen::Vector4d _x4d(_x(0), _x(1), _x(2), sTimeInterval_);
          Eigen::MatrixXd _c1 = sA_ * _x4d;
          for(int j=0; j<rowsA; ++j){
            F[1+i*rowsA+j] = _c1(j);
          }
        }
      }

      //C2
      F[1+fs.size()*rowsA] = (-1.0) * x[3] * minDis;
      //C3
      F[1+fs.size()*rowsA+1] = (x[4]*x[4] + x[5]*x[5] + x[6]*x[6] + x[7]*x[7]);

#ifdef V2D
      //planar formation
      F[1+fs.size()*rowsA+2] = x[5];
      F[1+fs.size()*rowsA+3] = x[6];
#endif


    }


    // gradients
    if ( *needG > 0 ) {
      //derivative of objcetive func
      G[0] = 2 * sWT_ * xx0;
      G[1] = 2 * sWT_ * xx1;
      G[2] = 2 * sWT_ * xx2;
      G[3] = 2 * sWS_ * xx3;
      G[4] = 2 * sWQ_ * xx4;
      G[5] = 2 * sWQ_ * xx5;
      G[6] = 2 * sWQ_ * xx6;
      G[7] = 2 * sWQ_ * xx7;

      //derivative of constraints
      //C1
      for(int i=0; i<fs.size(); ++i){
        if(rowsA>0){
          double _s;
          Eigen::Vector4d _q;
          Eigen::Vector3d _f;
          _s = x[3];
          _q << x[4], x[5], x[6], x[7];
          _f = fs[i];
          // jacobian of T
          Eigen::MatrixXd diffMatT = sA_.block(0,0,rowsA,3);
          // jacobian of S
          Eigen::Vector4d rot_0;
          rot_0 << drake::math::quatRotateVec(_q, _f), 0;
          Eigen::MatrixXd diffMatS = sA_ * rot_0;
          // jacobian of Q
          Eigen::MatrixXd drot = quatRotateVecDiff(_q, _f);
          Eigen::MatrixXd tmp = drot.block(0,0,3,4); // don't drot = drot.block(0,0,3,4);
          drot = tmp;
          Eigen::MatrixXd auxMat(4,4);
          auxMat << drot, Eigen::MatrixXd::Zero(1,4);
          Eigen::MatrixXd diffMatQ = _s * sA_ * auxMat; // should be _s rather than sS_
          // jacobian
          Eigen::MatrixXd diffMat(rowsA, 8);
          diffMat << diffMatT, diffMatS, diffMatQ;
          // assign
          for(int j=0; j<rowsA; ++j){
            for(int z=0; z<8; ++z){
              G[8+(i*rowsA+j)*8+z] = diffMat(j,z);
            }
          }
        }
      }

      //C2
      int base = (1 + fs.size()*rowsA) * 8;
      for(int i=0; i<8; ++i){
        if(i==3){
          G[base+i] = -minDis;
        }
        else{
          G[base+i] = 0;
        }
      }

      //C3
      base = (1 + fs.size()*rowsA + 1) * 8;
      for(int i=0; i<8; ++i){
        if(i>=4){
          G[base+i] = 2 * x[i];
        }
        else{
          G[base+i] = 0;
        }
      }

#ifdef V2D
      //planar formation
      base = (1 + fs.size()*rowsA + 2) * 8;
      for(int i=0; i<8; ++i){
        if(i==5){
          G[base+i] = 1;
        }
      }
      base = (1 + fs.size()*rowsA + 3) * 8;
      for(int i=0; i<8; ++i){
        if(i==6){
          G[base+i] = 1;
        }
      }
#endif


    }
  }

  // return minimal loss
  // the optimal param is passed through param
  double OptimalFormation::optimalDeviation(const Formation &formation, Vector8d &param){
    // static variables required initializing
    if(!sInitialized_){
      throw OptimalFormationError();
    }

    //SNOPT
    sFormation_ = formation;
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
#ifndef V2D
    int neF   =  1 + rowsA * fs.size() + 2;
#else
    //planar formation
    int neF   =  1 + rowsA * fs.size() + 4;
#endif

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

    // t
    x[0] = sG_(0);
    x[1] = sG_(1);
    x[2] = sG_(2);
    // s
    x[3] = double(2) * radius / minDis;
    // q
    srand(time(NULL));
    Eigen::Vector4d qIni = sQ_ + 0.1 * (rand() % 4 + 1) * Eigen::MatrixXd::Ones(4,1);
    //Eigen::Vector4d qIni = sQ_;
    qIni /= qIni.norm();
    x[4] = qIni(0);
    x[5] = qIni(1);
    x[6] = qIni(2);
    x[7] = qIni(3);

    // set the upper and lower bounds of x
    for(int i=0; i<n; ++i){
      //x[i] = 0.1;
      xlow[i] = -INFI;
      xupp[i] = INFI;
      xstate[i] = 0;
    }

    // set the upper and lower bounds of F
    // [0]         : obj
    // [1-#v*#rows]: C1
    // [#v*#rows+1]: C2
    // [#v*#rows+2]: C3
    // obj func
    Flow[0] = -INFI;
    Fupp[0] = INFI;
    Fmul[0] = 0;
    // C1
    for(int i=0; i<fs.size(); ++i){
      for(int j=0; j<rowsA; ++j){
        Flow[1+i*rowsA+j] = -INFI;
        Fupp[1+i*rowsA+j] = sB_(j);
        Fmul[1+i*rowsA+j] = 0;
      }
    }
    // C2
    Flow[1+fs.size()*rowsA] = -INFI;
    Fupp[1+fs.size()*rowsA] = (-2.0) * radius;
    Fmul[1+fs.size()*rowsA] = 0;
    // C3
    Flow[1+fs.size()*rowsA+1] = 1.0;
    Fupp[1+fs.size()*rowsA+1] = 1.0;
    Fmul[1+fs.size()*rowsA+1] = 0;

#ifdef V2D
    //planar formation
    Flow[1+fs.size()*rowsA+2] = 0;
    Fupp[1+fs.size()*rowsA+2] = 0;
    Fmul[1+fs.size()*rowsA+2] = 0;
    Flow[1+fs.size()*rowsA+3] = 0;
    Fupp[1+fs.size()*rowsA+3] = 0;
    Fmul[1+fs.size()*rowsA+3] = 0;
#endif

    // set the indicator of G
    for(int i=0; i<neF; ++i){
      for(int j=0; j<n; ++j){
        iGfun[i*n+j] = i;
        jGvar[i*n+j] = j;
      }
    }
    neG = lenG;

    // A related variables must be provided when defining derivative
    int lenA   = lenG;
    int *iAfun = new int[lenA];
    int *jAvar = new int[lenA];
    double *A  = new double[lenA];
    int neA = 0;

    // Load the data for ToyProb ...
    ToyProb.setProbName    ("uav");
    //ToyProb.setPrintFile   ( "uav.out" );
    ToyProb.setProblemSize ( n, neF );
    ToyProb.setObjective   ( ObjRow, ObjAdd );
    ToyProb.setA           ( lenA, neA, iAfun, jAvar, A ); // mush be set when providing derivative
    ToyProb.setX           ( x, xlow, xupp, xmul, xstate );
    ToyProb.setF           ( F, Flow, Fupp, Fmul, Fstate );
    ToyProb.setG           ( lenG, neG, iGfun, jGvar );
    ToyProb.setUserFun     ( sUsrfg_ );      // Sets the usrfun that supplies G and F.
    //ToyProb.setSpecsFile   ( "sntoya.spc" );
    ToyProb.setIntParameter( "Derivative option", 1 );
    ToyProb.setIntParameter( "Major Iteration limit", 250 );
    ToyProb.setIntParameter( "Verify level ", 3 );
    //suppress output
    //    ToyProb.setIntParameter( "Major print level ", 0 );
    //ToyProb.setIntParameter( "Summary file ", 0 );
    //ToyProb.setIntParameter( "Verify level ", 0 ); // implement of quatRotation gradient is not accurate
    ToyProb.solve          ( Cold );

    double res = F[ObjRow];
    for(int i=0; i<n; ++i){
      param(i) = x[i];
    }

        for (int i = 0; i < n; i++ ){
          cout << "x = " << x[i] << " xstate = " << xstate[i] << endl;
        }
    //    for (int i = 0; i < neF; i++ ){
    //      cout << "F = " << F[i] << " Fstate = " << Fstate[i] << endl;
    //    }
    cout << "C2: " << F[neF-1-1] << endl;
    cout << "C3: " << F[neF-1] << endl;



    delete []iGfun;  delete []jGvar;

    delete []x;      delete []xlow;   delete []xupp;
    delete []xmul;   delete []xstate;

    delete []F;      delete []Flow;   delete []Fupp;
    delete []Fmul;   delete []Fstate;

    return res;

  }

  // return index of optimal formation
  // deviation is passed through param
  int OptimalFormation::optimalFormation(Vector8d &param){
    int res;
    double minLoss = INT_MAX;
    for(int i=0; i<formations_.size(); ++i){
      Vector8d currParam;
      double currLoss = optimalDeviation(formations_[i], currParam);

      if(currLoss < minLoss){
        res = i;
        param = currParam;
        minLoss = currLoss;
      }
    }
    return res;
  }

}
