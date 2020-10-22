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

Solvenu::Solvenu(){countlimit=10000;limitfl=LIMITOFF;}
Solvenu::~Solvenu(){}

void Solvenu::setcountlimit(long a){countlimit = a;}

void Solvenu::setlimit(VectorXd uplimit,VectorXd lowlimit){
    upperlimit = uplimit;
    lowerlimit = lowlimit;
    limitfl = LIMITON;
}

int Solvenu::checklimit(VectorXd &x){
    if(limitfl==LIMITOFF){return 0;}
    int fl=0;
    for(int ii=0;ii<x.size();ii++){
        //errval(ii) = step(x(ii) - upperlimit(ii))*abs(x(ii) - upperlimit(ii)) - step( - x(ii) + lowerlimit(ii))*abs( - x(ii) + lowerlimit(ii)); 
        if(x(ii)> upperlimit(ii)){x(ii) = upperlimit(ii);fl++;}
        else if(x(ii)< lowerlimit(ii)){x(ii) = lowerlimit(ii);fl++;}
    }
    return fl;
}

VectorXd Solvenu::sigmoidlimit(VectorXd &x,double alpha){
    VectorXd ans = VectorXd::Zero(1);
    if(limitfl==LIMITOFF){return ans;}
    for(int ii=0;ii<x.size();ii++){
        ans(0) += sigmoid(x(ii)-upperlimit(ii),alpha);
    }
    return ans;
}

void Solvenu::settargetfx(VectorXd tfx){targetfx = tfx;}
VectorXd Solvenu::gettargetfx(){return targetfx;}

VectorXd Solvenu::function(VectorXd x){
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
    checklimit(x);
    VectorXd buf = funcorg(x);
    VectorXd trgfx(buf.size());
    return  funcorg(x) - targetfx;
}

VectorXd Solvenu::solve(VectorXd intx){
    checklimit(intx);
    x = intx;
    long count = 0;
    VectorXd dx = intx;
    MatrixXd buf;
    VectorXd chk;
    if(0){//x.size()==targetfx.size()){
        while(1){
            checklimit(x);
            dx = x;
            buf = diffvec(x,this);
            if(buf.determinant()<0.000001){ 
                std::cout<< "determin 0"<< std::endl;exit(0);
                }
            x = x - inv(diffvec(x,this))*functionerror(x);
            chk = MatrixXd::Ones(1,x.size())*absmat(x - dx);
            if (chk(0) < 0.0000000001) break;
            if(count>100000){std::cout <<"CAUTIONCAUTIONCAUTIONCAUTIONCAUTION step is more than 10000 CAUTIONCAUTIONCAUTIONCAUTIONCAUTIONC"<<std::endl;PRINT_MAT(functionerror(x));break;}
            count++;
        }
    }else{
        while(1){
            //checklimit(x);
            dx = x;
            MatrixXd baka =  diffvec(x,this);
            JacobiSVD<MatrixXd> svd(diffvec(x,this), ComputeThinU|ComputeThinV);
            x = x - svd.solve(functionerror(x));
            chk = MatrixXd::Ones(1,x.size())*absmat(x - dx) + sigmoidlimit(x,1000);
            if(count>countlimit){std::cout <<"CAUTIONCAUTIONCAUTIONCAUTIONCAUTION step is more than 10000 CAUTIONCAUTIONCAUTIONCAUTIONCAUTIONC"<<std::endl;PRINT_MAT(functionerror(x));break;}
            if (chk(0) < 0.0000000001d) break;
            count++;
        }

    }
    return x;
}