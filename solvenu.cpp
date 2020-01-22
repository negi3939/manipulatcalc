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
    PRINT_MAT(x);
    PRINT_MAT(functionerror(x));
    return functionerror(x);
}

VectorXd Solvenu::funcorg(VectorXd x){
    VectorXd ans(2);
    MatrixXd aA(ans.size(),x.size());
    aA << 1,0,0,1;
    ans = aA*x;
    return ans;
}

VectorXd Solvenu::functionerror(VectorXd x){
    VectorXd buf = funcorg(x);
    VectorXd trgfx(buf.size());
    return  funcorg(x) - targetfx;
}




VectorXd Solvenu::solve(VectorXd intx){
    x = intx;
    VectorXd dx = intx;
    VectorXd chk;
    if(x.size()==targetfx.size()){
        while(1){
            dx = x;
            x = x - inv(diffvec(x,this))*functionerror(x);
            PRINT_MAT(x);
            PRINT_MAT(functionerror(x));
            chk = MatrixXd::Ones(1,x.size())*absmat(x - dx);
            if (chk(0) < 0.0000000001) break;
        }
    }else{
        while(1){
            dx = x;
            MatrixXd baka =  diffvec(x,this);
            PRINT_MAT(baka);
            JacobiSVD<MatrixXd> svd(diffvec(x,this), ComputeThinU|ComputeThinV);
            PRINT_MAT(svd.solve(functionerror(x)));
            x = x - svd.solve(functionerror(x));
            std::cout << "svd mode" << std::endl;
            PRINT_MAT(x);
            PRINT_MAT(functionerror(x));
            chk = MatrixXd::Ones(1,x.size())*absmat(x - dx);
            if (chk(0) < 0.0000000001) break;
        }

    }
    
    return x;
}