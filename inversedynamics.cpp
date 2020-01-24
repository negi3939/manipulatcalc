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

#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl
using namespace Mymath;

class invdSolvenu : public invkSolvenu {
  protected:
    Matrix4d *aTt;
    Vector3d *rra;
    Vector3d *ppa;
    Vector3d *zz;
    MatrixXd *jacobi;
  public:
    invdSolvenu();
    invdSolvenu(int num);
    VectorXd funcorg(VectorXd x) override;
    VectorXd getvel(VectorXd x);
    ~invdSolvenu();
};

invdSolvenu::invdSolvenu():invkSolvenu(){
    aTt = new Matrix4d[jointnum];
    rra = new Vector3d[jointnum];
    ppa = new Vector3d[jointnum];
    zz = new Vector3d[jointnum];
    jacobi = new MatrixXd;
    *jacobi= MatrixXd::Zero(6,jointnum);
    rra[0] = Vector3d::Zero(3,1);
    zz[0] << 0.0d,0.0d,1.0d;
}
invdSolvenu::invdSolvenu(int num):invkSolvenu(num){
    aTt = new Matrix4d[jointnum];
    rra = new Vector3d[jointnum];
    ppa = new Vector3d[jointnum];
    zz = new Vector3d[jointnum];
    jacobi = new MatrixXd;
    *jacobi = MatrixXd::Zero(6,jointnum);
    rra[0] = Vector3d::Zero(3,1);
    zz[0] << 0.0d,0.0d,1.0d;
}
invdSolvenu::~invdSolvenu(){delete jacobi;delete[] ppa;delete[] rra;delete[] aTt;delete[] zz;}

VectorXd invdSolvenu::funcorg(VectorXd x){
    int ii;
    VectorXd ans = VectorXd::Zero(6);
    Vector3d rrend;
    Matrix4d buff = aA[0];
    aTt[0] = buff;
    for(ii=1;ii<jointnum;ii++){
        buff = buff*aA[ii];
        aTt[ii] = buff;
    }
    rrend = aTt[jointnum-1].block(0,3,3,1);
    ppa[0] = rrend;
    jacobi->block(0,0,3,1) = zz[0].cross(ppa[0]);
    jacobi->block(3,0,3,1) = zz[0];
    for(ii=1;ii<jointnum;ii++){
        rra[ii] = aTt[ii-1].block(0,3,3,1);
        zz[ii] = aTt[ii-1].block(0,2,3,1);
        ppa[ii] = rrend - rra[ii];
        jacobi->block(0,ii,3,1) = zz[ii].cross(ppa[ii]);
        jacobi->block(3,ii,3,1) = zz[ii];
    }
    ans = (*jacobi)*x;
    return ans;
}

VectorXd invdSolvenu::getvel(VectorXd x){
    VectorXd vel = solve(x);
    return vel;
}