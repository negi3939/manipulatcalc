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

void Solvenu::unsetlimit(){
    limitfl = LIMITOFF;
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

VectorXd Solvenu::penaltyfunc(VectorXd &x){
    VectorXd ans =VectorXd::Zero(x.size()) ;
    for(int ii=0;ii<x.size();ii++){
        ans(ii) += step(x(ii)-upperlimit(ii))*(x(ii)-upperlimit(ii))*(x(ii)-upperlimit(ii));
        ans(ii) += step(-x(ii)+lowerlimit(ii))*(-x(ii)+lowerlimit(ii))*(-x(ii)+lowerlimit(ii));
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
    //checklimit(x);
    VectorXd buf = funcorg(x);
    VectorXd trgfx(buf.size());
    if(limitfl==0){ 
        return  funcorg(x) - targetfx;
    }else{
        //PRINT_MAT(funcorg(x));
        //PRINT_MAT(targetfx);
        //PRINT_MAT(funcorg(x)- targetfx);
        //exit(0);
        MatrixXd oone = 100.0d*MatrixXd::Ones(x.size(),targetfx.size());
        return  funcorg(x) - targetfx + oone*penaltyfunc(x);
    }
}

VectorXd Solvenu::solve(VectorXd intx,SolvFLAG slflag=NEWTON){
    switch (slflag){
    case NEWTON:
        return newtonsolve(intx);
        break;
    case STEEPEST:
        return steepsetdescentsolve(intx);
        break;
    default:
        return intx;
        break;
    }
}

VectorXd Solvenu::newtonsolve(VectorXd intx){
    x = intx;
    long count = 0;
    VectorXd dx = intx;
    MatrixXd buf;
    VectorXd chk;
    while(1){
        dx = x;
        MatrixXd baka =  diffvec(x,this);
        JacobiSVD<MatrixXd> svd(diffvec(x,this), ComputeThinU|ComputeThinV);
        x = x - svd.solve(functionerror(x));//limit ここを書き換える必要がある
        //PRINT_MAT(x);
        chk = MatrixXd::Ones(1,x.size())*absmat(x - dx);// + sigmoidlimit(x,1000);
        if(count>countlimit){std::cout <<"CAUTIONCAUTIONCAUTIONCAUTIONCAUTION step is more than 10000 CAUTIONCAUTIONCAUTIONCAUTIONCAUTIONC"<<std::endl;PRINT_MAT(functionerror(x));break;}
        if (functionerror(x).norm() < 0.000001d) break;
        count++;
    }
    return x;
}

VectorXd Solvenu::steepsetdescentsolve(VectorXd intx){
    long count = 0;
    VectorXd x = intx;
    MatrixXd diffv;
    double gold_r = 0.6180339887d;
    double bottom_alpha,top_alpha,alpha1,alpha2;
    MatrixXd s;
    while(1){
        diffv =  diffnorm(x,this);
        if(functionerror(x).norm()<0.00001d){break;}
        //std::cout << "norm is " <<functionerror(x).norm() << std::endl;
        s = -diffv.transpose();
        bottom_alpha=0.0d;
        top_alpha=0.1d;
        alpha1 = bottom_alpha + (1.0d -gold_r)*(top_alpha - bottom_alpha);
        alpha2 = bottom_alpha + gold_r*(top_alpha - bottom_alpha);
        while(1){
            if(functionerror(x + alpha1*s).norm()<functionerror(x + alpha2*s).norm()){
                top_alpha = alpha2;
                alpha2 = alpha1;
                alpha1 = bottom_alpha + (1.0d -gold_r)*(top_alpha - bottom_alpha);
            }else{
                bottom_alpha = alpha1;
                alpha1 = alpha2;
                alpha2 = bottom_alpha + gold_r*(top_alpha - bottom_alpha);
            }
            if(count>countlimit){std::cout <<"CAUTIONCAUTIONCAUTIONCAUTIONCAUTION step is more than 10000 CAUTIONCAUTIONCAUTIONCAUTIONCAUTIONC"<<std::endl;PRINT_MAT(functionerror(x));break;}
            //std::cout << "bottom_alpha : "<< bottom_alpha << " top_alpha : "<< top_alpha << std::endl;
            if(std::abs(bottom_alpha - top_alpha) <0.00001d){break;}
        }
        x = x + 0.5d*(bottom_alpha+top_alpha)*s;
    }
    return x;
}