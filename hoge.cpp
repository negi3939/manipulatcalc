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

VectorXd function(VectorXd x){
    VectorXd ans(2);
    MatrixXd aA(2,x.size());
    aA << 1,2,3,4;
    ans = aA*x;
    return ans;
}

VectorXd functionerror(VectorXd x){
    VectorXd trgfx(2);
     trgfx << 2,4;
    return  function(x) -trgfx;
}

MatrixXd diffvec(VectorXd x,VectorXd (*f)(VectorXd)){
    int ii;
    VectorXd fx = f(x);
    MatrixXd ans(fx.size(),x.size());
    MatrixXd bef(fx.size(),x.size());
    MatrixXd aft(fx.size(),x.size());
    double delta = 0.00000001;
    VectorXd deltax(x.size());
    for(ii=0;ii<x.size();ii++){
        deltax = VectorXd::Zero(x.size());
        deltax(ii) =  delta; 
        bef.block(0,ii,x.size(),1) = (fx-f(x-deltax))/delta;
        aft.block(0,ii,x.size(),1) = (f(x+deltax)-fx)/delta;
    }
    ans = (bef+aft)/2.0;
    if(ans.determinant()==0){return MatrixXd::Identity(fx.size(),x.size());}
    return ans;
}

int main(){
    Vector2d x = VectorXd::Zero(2);
    VectorXd dx = x;
    VectorXd chk;
    PRINT_MAT(function(x));
        while (1){
        dx = x;
        x = x - inv(diffvec(x,functionerror))*functionerror(x);
        PRINT_MAT(x);
        PRINT_MAT(functionerror(x));
        chk = MatrixXd::Ones(1,x.size())*absmat(x - dx);
        if (chk(0) < 0.0000000001) break;
    }
    PRINT_MAT(function(x));
    std::cout << "success calculated" << std::endl; 
    return 0;
}