#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <unistd.h>
#include <getopt.h>
#include <stdlib.h>
#include <stdio.h>
#include <string>
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

#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl
using namespace Mymath;

class hogeSolvenu :public Solvenu{
    protected:
    public:
        VectorXd funcorg(VectorXd x) override;
};

VectorXd hogeSolvenu::funcorg(VectorXd x){
    VectorXd ans(2);
    MatrixXd aA(ans.size(),x.size());
    ans(0) = x(0)*x(0) + x(1)*x(1) + x(2);
    ans(1) = x(0) + x(1)*x(1) + x(2)*x(2);
    return ans;
}

class Dhparam{
    protected:

    public:
};

class maniSolvenu : public Solvenu {
  protected:
    int jointnum;
    MatrixXd *aA;
    double *aal;
    double *alp;
    double *dis;
  public:
    manipulator();
};

maniSolvenu::manipulator(){
    aA = new MatrixXd[jointnum];
    aal = new double[jointnum];
    alp = new double[jointnum];
    dis = new double[jointnum];
}


int main(){
    maniSolvenu mani;

    VectorXd x = VectorXd::Zero(3);
    VectorXd dx = x;
    VectorXd chk;
    VectorXd tarfx(2);
    tarfx << 8,14;
    hogeSolvenu eqex;
    eqex.settargetfx(tarfx);
    x = eqex.solve(x);
    PRINT_MAT(eqex.funcorg(x));
    std::cout << "success calculated" << std::endl; 
    return 0;
}