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

#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl
using namespace Mymath;

class Solvenu :public Funcvec{
    protected:
        VectorXd targetfx;
        VectorXd x;
    public:
        void settargetfx(VectorXd tfx);
        VectorXd gettargetfx();
        VectorXd function(VectorXd x) override;
        virtual VectorXd funcorg(VectorXd x);
        VectorXd functionerror(VectorXd x);
        VectorXd solve(VectorXd intx);
};

void Solvenu::settargetfx(VectorXd tfx){targetfx = tfx;}
VectorXd Solvenu::gettargetfx(){return targetfx;}

VectorXd Solvenu::function(VectorXd x){
    std::cout << "this is Slolvene" << std::endl; 
    PRINT_MAT(x);
    PRINT_MAT(functionerror(x));
    return functionerror(x);
}

VectorXd Solvenu::funcorg(VectorXd x){
    VectorXd ans(2);
    MatrixXd aA(2,x.size());
    aA << 1,2,3,4;
    ans = aA*x;
    return ans;
}

VectorXd Solvenu::functionerror(VectorXd x){
    VectorXd trgfx(2);
    return  funcorg(x) - targetfx;
}

VectorXd Solvenu::solve(VectorXd intx){
    x = intx;
    VectorXd dx = intx;
    VectorXd chk;
    while (1){
        dx = x;
        x = x - inv(diffvec(x,this))*functionerror(x);
        PRINT_MAT(x);
        PRINT_MAT(functionerror(x));
        chk = MatrixXd::Ones(1,x.size())*absmat(x - dx);
        if (chk(0) < 0.0000000001) break;
    }
    return x;
}

int main(){
    Vector2d x = VectorXd::Zero(2);
    VectorXd dx = x;
    VectorXd chk;
    VectorXd tarfx(2);
    tarfx << 2,4;
    Solvenu eqex;
    eqex.settargetfx(tarfx);
    x = eqex.solve(x);
    PRINT_MAT(eqex.funcorg(x));
    std::cout << "success calculated" << std::endl; 
    return 0;
}