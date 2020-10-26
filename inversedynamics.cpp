#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <unistd.h>
#include <getopt.h>
#include <stdlib.h>
#include <string>
#include <stdio.h>
#include <stdint.h>
#include <vector>
#include <math.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/times.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <bitset>
#include <termios.h>
#include <pthread.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/LU>
#include "mymath.h"
#include "solvenu.h"
#include "inversekinematics.h"
#include "inversedynamics.h"

invdSolvenu::invdSolvenu():invkSolvenu(){}
invdSolvenu::invdSolvenu(int num):invkSolvenu(num){}
invdSolvenu::~invdSolvenu(){delete jacobi;delete[] ppa;delete[] rra;delete[] aTt;delete[] zz;}

VectorXd invdSolvenu::funcorg(VectorXd x){
    calcaTt();
    calcjacobi();
    return (*jacobi)*x;
}


VectorXd invdSolvenu::gettau(VectorXd f,VectorXd mom){
    calcaTt();
    calcjacobi();
    VectorXd fm,tau;
    fm.resize(f.size()+mom.size(),1);
    fm.block(0,0,f.size(),1) = f;
    fm.block(f.size(),0,mom.size(),1) = mom;
    tau = jacobi->transpose()*fm;
    return tau;
}

void invdSolvenu::calcforce(VectorXd tau,Vector3d &f,Vector3d &mom){
    calcaTt();
    calcjacobi();
    VectorXd fmom(6);
    fmom.block(0,0,3,1) = f;
    fmom.block(3,0,3,1) = mom;
    /*JacobiSVD<MatrixXd> svd(jacobi->transpose(),ComputeThinU|ComputeThinV);
    fmom = svd.solve(tau);*/
    fmom = pseudo_inv(jacobi->transpose())*tau;
    f = fmom.block(0,0,3,1);
    mom = fmom.block(3,0,3,1);//動作未確認*/
}

VectorXd invdSolvenu::getvel(VectorXd x){
    VectorXd vel = solve(x);
    return vel;
}

#if defined(ID_IS_MAIN)
VectorXd forward_dynamics(invdSolvenu *maninvd,VectorXd &angle,Vector3d &forcev,Vector3d &momentv){
    VectorXd tau;
    angle(0) = -0.25d*M_PI;
    angle(1) = -0.25d*M_PI;
    angle(2) = -0.25d*M_PI;
    angle(3) = -0.25d*M_PI;
    angle(4) = -0.25d*M_PI;
    angle(5) = -0.25d*M_PI;
    angle(6) = -0.25d*M_PI;
    maninvd->calcaA(angle);
    forcev(0) = 1.0d;
    forcev(1) = 1.0d;
    forcev(2) = 1.0d;
    momentv(0) = 0.0d;
    momentv(1) = 0.0d;
    momentv(2) = 0.0d;
    tau = maninvd->gettau(forcev,momentv);
    PRINT_MAT(tau);
    return tau;
}

void inverse_dynamics(invdSolvenu *maninvd,VectorXd &angle,VectorXd &ctauv,Vector3d &forcev,Vector3d &momentv){
    double torqconstant = 3.70d;
    angle(0) = -0.25d*M_PI;
    angle(1) = -0.25d*M_PI;
    angle(2) = -0.25d*M_PI;
    angle(3) = -0.25d*M_PI;
    angle(4) = -0.25d*M_PI;
    angle(5) = -0.25d*M_PI;
    angle(6) = -0.25d*M_PI;
    maninvd->calcaA(angle);
    ctauv(0) = 1.0d;
    ctauv(1) = 1.0d;
    ctauv(2) = 1.0d;
    ctauv(3) = 1.0d;
    ctauv(4) = 1.0d;
    ctauv(5) = 1.0d;
    ctauv(6) = 1.0d;
    maninvd->calcforce(torqconstant*ctauv,forcev,momentv);
    PRINT_MAT(forcev);
    PRINT_MAT(momentv);
}

int main(){
    int ii,jointn = 7;
    invdSolvenu *maninvd;
    maninvd = new invdSolvenu(jointn);
    /*RT CRANE*/
    maninvd->setdhparameter(0,0.0d*M_PI,0.0d,0.064d,-0.5d*M_PI);//(int num,double thoff,double aa,double di,double alph);
    maninvd->setdhparameter(1,0.0d*M_PI,0.0d,0.0d,0.5d*M_PI);//(int num,double thoff,double aa,double di,double alph);
    maninvd->setdhparameter(2,0.0d*M_PI,0.0d,0.065d+0.185d,-0.5d*M_PI);//(int num,double thoff,double aa,double di,double alph);
    maninvd->setdhparameter(3,0.0d*M_PI,0.0d,0.0d,0.5d*M_PI);//(int num,double thoff,double aa,double di,double alph);
    maninvd->setdhparameter(4,0.0d*M_PI,0.0d,0.121d+0.129d,-0.5d*M_PI);//(int num,double thoff,double aa,double di,double alph);
    maninvd->setdhparameter(5,0.0d*M_PI,0.0d,0.0d,0.5d*M_PI);//(int num,double thoff,double aa,double di,double alph);
    maninvd->setdhparameter(6,0.0d*M_PI,0.0d,0.019d+0.084d,0.0d);//(int num,double thoff,double aa,double di,double alph);
    /**/
    VectorXd angle = VectorXd::Zero(jointn);//joint angle
    VectorXd ctauv = VectorXd::Zero(jointn);//current
    Matrix4d mattheta = MatrixXd::Identity(4,4);//回転変位行列
    Vector3d forcev,momentv;//手先力,手先モーメント
    /*test*/
    forward_dynamics(maninvd,angle,forcev,momentv);//calc FD test
    inverse_dynamics(maninvd,angle,ctauv,forcev,momentv);//calc ID test

    delete maninvd;
    return 0;
}
#endif