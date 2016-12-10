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

double radius = 5;
double minDis = 40000;

Eigen::Vector3d sG_(-1,-2,0);
double sS_ = 1024;
Eigen::Vector4d sQ_(1,0,0,0);
double sWT_ = 2;
double sWS_ = 3;
double sWQ_ = 4;
double pref = 5;

void toyusrfg_( int    *Status, int *n,    double x[],
    int    *needF,  int *neF,  double F[],
    int    *needG,  int *neG,  double G[],
    char      *cu,  int *lencu,
    int    iu[],    int *leniu,
    double ru[],    int *lenru )
{
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



  if ( *needF > 0 ) {
    F[0] = sWT_ * ttSquare + sWS_ * ssSquare + sWQ_ * qqSquare + pref;
    F[1] = (x[0]*x[0] + x[1]*x[1] + x[2]*x[2] + x[3]*x[3] + x[4]*x[4] + x[5]*x[5] + x[6]*x[6] + x[7]*x[7]);
  }

  if ( *needG > 0 ) {
    G[0] = 2 * sWT_ * xx0;
    G[1] = 2 * sWT_ * xx1;
    G[2] = 2 * sWT_ * xx2;
    G[3] = 2 * sWS_ * xx3;
    G[4] = 2 * sWQ_ * xx4;
    G[5] = 2 * sWQ_ * xx5;
    G[6] = 2 * sWQ_ * xx6;
    G[7] = 2 * sWQ_ * xx7;

    int base = (1) * 8;
    for(int i=0; i<8; ++i){
      //   if(i>=4){
      G[base+i] = 2 * x[i];
      //   }
      //   else{
      //     G[base+i] = 0;
      //   }
    }


//    G[0] = 2 * sWT_ * xx0;
//    G[2] = 2 * sWT_ * xx1;
//    G[4] = 2 * sWT_ * xx2;
//    G[6] = 2 * sWS_ * xx3;
//    G[8] = 2 * sWQ_ * xx4;
//    G[10] = 2 * sWQ_ * xx5;
//    G[12] = 2 * sWQ_ * xx6;
//    G[14] = 2 * sWQ_ * xx7;
//    int base = (1) * 8;
//    for(int i=0; i<8; ++i){
//      //   if(i>=4){
//      G[i*2+1] = 2 * x[i];
//      //   }
//      //   else{
//      //     G[base+i] = 0;
//      //   }
//    }

  }
}

int main(int argc, char** argv) {
  //------------------------------SNOPT part------------------------------

  //  int rowsA = A.rows();
  //  int colsA = A.cols();

  snoptProblemA ToyProb;

  // Allocate and initialize;
  int n     =  8;
  int neF   =  2; // #constrains + obj_func

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
  Flow[1] = 1;
  Fupp[1] = 1;

  // set the indicator of G
  for(int i=0; i<neF; ++i){
    for(int j=0; j<n; ++j){
      iGfun[i*n+j] = i;
      jGvar[i*n+j] = j;
      cout << "G[" << i*n+j << "] = " << i << ", " << j << endl;
    }
  }
  neG = lenG;


  int lenA   = lenG;
  int *iAfun = new int[lenA];
  int *jAvar = new int[lenA];
  double *A  = new double[lenA];
  int neA = 0;


  // Load the data for ToyProb ...
  ToyProb.setProbName    ("uav");
  ToyProb.setPrintFile   ( "uav.out" );
  ToyProb.setProblemSize ( n, neF );
  ToyProb.setObjective   ( ObjRow, ObjAdd );
  ToyProb.setX           ( x, xlow, xupp, xmul, xstate );
  ToyProb.setF           ( F, Flow, Fupp, Fmul, Fstate );

  ToyProb.setA           ( lenA, neA, iAfun, jAvar, A );

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
