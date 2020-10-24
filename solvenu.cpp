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

VectorXd Solvenu::penaltyfunc(VectorXd &x){
    VectorXd ans =VectorXd::Zero(x.size()) ;
    for(int ii=0;ii<x.size();ii++){
        ans(ii) += step(x(ii)-upperlimit(ii))*(x(ii)-upperlimit(ii))*(x(ii)-upperlimit(ii));
        ans(ii) += step(-x(ii)+lowerlimit(ii))*(-x(ii)+lowerlimit(ii))*(x(ii)-upperlimit(ii));
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
        return  funcorg(x) - targetfx;// + oone*penaltyfunc(x);
    }
}

VectorXd Solvenu::solve(VectorXd intx){
    //checklimit(intx);
    x = intx;
    long count = 0;
    VectorXd dx = intx;
    MatrixXd buf;
    VectorXd chk;
    if(limitfl){//limitON
        while(1){
            dx = x;
            MatrixXd baka =  diffvec(x,this);
            JacobiSVD<MatrixXd> svd(diffvec(x,this), ComputeThinU|ComputeThinV);
            x = x - svd.solve(functionerror(x));// - 0.01d*sigmoid(x-upperlimit,1000) + 0.01d* sigmoid(lowerlimit-x,1000);//limit ここを書き換える必要がある
            chk = MatrixXd::Ones(1,x.size())*absmat(x - dx);// + sigmoidlimit(x,1000);
            if(count>countlimit){std::cout <<"CAUTIONCAUTIONCAUTIONCAUTIONCAUTION step is more than 10000 CAUTIONCAUTIONCAUTIONCAUTIONCAUTIONC"<<std::endl;/*PRINT_MAT(functionerror(x));*/break;}
            if (chk(0) < 0.000001d) {break;}
            count++;
        }
    }else{//limitOFF
        while(1){
            dx = x;
            MatrixXd baka =  diffvec(x,this);
            JacobiSVD<MatrixXd> svd(diffvec(x,this), ComputeThinU|ComputeThinV);
            x = x - svd.solve(functionerror(x));//limit ここを書き換える必要がある
            chk = MatrixXd::Ones(1,x.size())*absmat(x - dx);// + sigmoidlimit(x,1000);
            if(count>countlimit){std::cout <<"CAUTIONCAUTIONCAUTIONCAUTIONCAUTION step is more than 10000 CAUTIONCAUTIONCAUTIONCAUTIONCAUTIONC"<<std::endl;PRINT_MAT(functionerror(x));break;}
            if (chk(0) < 0.000001d) break;
            count++;
        }

    }
    //checklimit(x);
    return x;
}

VectorXd Solvenu::solve(VectorXd intx,double thval){
    //checklimit(intx);
    x = intx;
    long count = 0;
    VectorXd dx = intx;
    MatrixXd buf;
    VectorXd chk;
    if(limitfl){//limitON
        while(1){
            dx = x;
            MatrixXd baka =  diffvec(x,this);
            JacobiSVD<MatrixXd> svd(diffvec(x,this), ComputeThinU|ComputeThinV);
            x = x - svd.solve(functionerror(x));// - 0.01d*sigmoid(x-upperlimit,1000) + 0.01d* sigmoid(lowerlimit-x,1000);//limit ここを書き換える必要がある
            chk = MatrixXd::Ones(1,x.size())*absmat(x - dx);// + sigmoidlimit(x,1000);
            if(count>countlimit){std::cout <<"CAUTIONCAUTIONCAUTIONCAUTIONCAUTION step is more than 10000 CAUTIONCAUTIONCAUTIONCAUTIONCAUTIONC"<<std::endl;/*PRINT_MAT(functionerror(x));*/break;}
            if (chk(0) < thval) {break;}
            count++;
        }
    }else{//limitOFF
        while(1){
            dx = x;
            MatrixXd baka =  diffvec(x,this);
            JacobiSVD<MatrixXd> svd(diffvec(x,this), ComputeThinU|ComputeThinV);
            x = x - svd.solve(functionerror(x));//limit ここを書き換える必要がある
            chk = MatrixXd::Ones(1,x.size())*absmat(x - dx);// + sigmoidlimit(x,1000);
            if(count>countlimit){std::cout <<"CAUTIONCAUTIONCAUTIONCAUTIONCAUTION step is more than 10000 CAUTIONCAUTIONCAUTIONCAUTIONCAUTIONC"<<std::endl;PRINT_MAT(functionerror(x));break;}
            if (chk(0) < thval) break;
            count++;
        }

    }
    //checklimit(x);
    return x;
}