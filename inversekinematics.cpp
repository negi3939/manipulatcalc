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
#include "inversekinematics.h"


invkSolvenu::invkSolvenu():Solvenu(){jointnum = 6;init();}
invkSolvenu::invkSolvenu(int num):Solvenu(){jointnum = num;init();}

void invkSolvenu::copy(const invkSolvenu &invk){
    int ii;
    setjointnum(invk.getjointnum());
    settime(invk.gettimead());
    for(ii=0;ii<jointnum;ii++){
        setdhparameter(ii,invk.getthetaoff(ii),invk.getaal(ii),invk.getdis(ii),invk.getalp(ii));
    }

}

void invkSolvenu::copy(invkSolvenu *invk){
    int ii;
    setjointnum(invk->getjointnum());
    settime(invk->gettimead());
    for(ii=0;ii<jointnum;ii++){
        setdhparameter(ii,invk->getthetaoff(ii),invk->getaal(ii),invk->getdis(ii),invk->getalp(ii));
    }

}

void invkSolvenu::setjointnum(int n){jointnum=n;}
void invkSolvenu::settime(const double &t){time = &t;}
void invkSolvenu::settime(double *t){time = t;}
int invkSolvenu::getjointnum(){return jointnum;}
double invkSolvenu::gettime(){return *time;}
double* invkSolvenu::gettimead(){return time;}
double invkSolvenu::getaal(int n){return aal[n];}
double invkSolvenu::getalp(int n){return alp[n];}
double invkSolvenu::getdis(int n){return dis[n];}
double invkSolvenu::getthetaoff(int n){return thetaoff[n];}

void invkSolvenu::init(){
    aA = new Matrix4d[jointnum];
    aAdis = new Matrix4d[jointnum];
    aAtheta = new Matrix4d[jointnum];
    aAthetaoff = new Matrix4d[jointnum];
    aAaal = new Matrix4d[jointnum];
    aAalp = new Matrix4d[jointnum];
    aal = new double[jointnum];
    alp = new double[jointnum];
    dis = new double[jointnum];
    thetaoff = new double[jointnum];
    time = new double;
    pre_time = new double;
    *time = 0.0d;
    *pre_time = 0.0d;
    for(int ii=0;ii<jointnum;ii++){
        aAdis[ii] = MatrixXd::Identity(4,4);
        aAthetaoff[ii] = MatrixXd::Identity(4,4);
        aAtheta[ii] = MatrixXd::Identity(4,4);
        aAaal[ii] = MatrixXd::Identity(4,4);
        aAalp[ii] = MatrixXd::Identity(4,4);
    }
}

VectorXd invkSolvenu::funcorg(VectorXd x){
    int ii;
    Vector3d pos;
    Vector4d qua;
    VectorXd ans(7);
    Matrix4d allA;
    calcaA(x,allA);
    qua = matrixtoquatanion(allA);
    pos = allA.block(0,3,3,1);
    ans.block(0,0,3,1) = pos;
    ans.block(3,0,4,1) = qua.block(0,0,4,1);
    return ans;
}

void invkSolvenu::calcaA(VectorXd x){
    for(int ii=0;ii<jointnum;ii++){
        aAtheta[ii](0,0) = cos(x(ii)+thetaoff[ii]);
        aAtheta[ii](0,1) = -sin(x(ii)+thetaoff[ii]);
        aAtheta[ii](1,0) = sin(x(ii)+thetaoff[ii]);
        aAtheta[ii](1,1) = cos(x(ii)+thetaoff[ii]);
        aA[ii] = aAdis[ii]*aAtheta[ii]*aAaal[ii]*aAalp[ii];
    }
}

void invkSolvenu::calcaA(VectorXd x,Matrix4d &reta){
    calcaA(x);
    Matrix4d allA = aA[0];
    for(int ii=1;ii<jointnum;ii++){
        allA = allA*aA[ii];
    }
    reta = allA;
}

void invkSolvenu::setdhparameter(int num,double thoff,double aa,double di,double alph){
    thetaoff[num] = thoff;
    aal[num] = aa;
    dis[num] = di;
    alp[num] = alph;
    aAdis[num](2,3) = dis[num];
    aAaal[num](0,3) = aal[num];
    aAalp[num](1,1) = cos(alp[num]);
    aAalp[num](1,2) = -sin(alp[num]);
    aAalp[num](2,1) = sin(alp[num]);
    aAalp[num](2,2) = cos(alp[num]);
    aAthetaoff[num](0,0) = cos(thetaoff[num]);
    aAthetaoff[num](1,0) = sin(thetaoff[num]);
    aAthetaoff[num](0,1) = -sin(thetaoff[num]);
    aAthetaoff[num](1,1) = cos(thetaoff[num]);
}

Vector4d invkSolvenu::matrixtoquatanion(Matrix4d mat){
    Vector4d ans;
    Matrix3d matthree;
    matthree = mat.block(0,0,3,3);
    Quaterniond qu(matthree);
	ans << qu.w(),qu.x(), qu.y(), qu.z();
	return ans;
}

Matrix3d invkSolvenu::quataniontomatrix(Vector4d qua){
    Quaterniond qu(qua(0),qua(1),qua(2),qua(3));
	return qu.matrix();
}

VectorXd invkSolvenu::getangle(VectorXd x){
    VectorXd ang = solve(x);
    VectorXd error = functionerror(ang);
    while(0){
        x = VectorXd::Random(jointnum,1);
        ang = solve(x);
        error = functionerror(ang);
        //PRINT_MAT(error);

    }
    VectorXd ans(jointnum);
    double buff;
    for(int ii=0;ii<jointnum;ii++){
        ans(ii) = atan2(sin(ang(ii)),cos(ang(ii)));
    }
    return ans;
    
}

invkSolvenu::~invkSolvenu(){
    delete[] aal;
    delete[] alp;
    delete[] dis;
    delete[] aAalp;
    delete[] aAaal;
    delete[] aAtheta;
    delete[] aAdis;
    delete[] aA;
    delete pre_time;
}

#if defined(IK_IS_MAIN)
void forward_kinematics(invkSolvenu *maninvk,VectorXd &angle,Matrix4d &mattheta){
    Vector4d qua;//クオータニオン
    angle(0) = -0.25d*M_PI;
    angle(1) = -0.25d*M_PI;
    angle(2) = -0.25d*M_PI;
    angle(3) = -0.25d*M_PI;
    angle(4) = -0.25d*M_PI;
    angle(5) = -0.25d*M_PI;
    angle(6) = -0.25d*M_PI;
    maninvk->calcaA(angle,mattheta);
    qua = maninvk->matrixtoquatanion(mattheta);
    std::cout << "xyz is \t" << mattheta(0,3) << " , " << mattheta(1,3) << " , "<< mattheta(2,3) << std::endl;
}

void inverse_kinematics(invkSolvenu *maninvk,VectorXd &angle,Matrix4d &mattheta){
    VectorXd uplimit(7);
    VectorXd lowlimit(7);
    uplimit <<    2.72271 ,  0.5*M_PI  ,   2.72271  ,        0 ,   2.72271 ,    0.5*M_PI ,   2.89725;
    lowlimit <<  -2.72271 , -0.5*M_PI  ,  -2.72271  , -2.79253 ,  -2.72271 ,   -0.5*M_PI ,  -2.89725;
    VectorXd targetx(7);//目標位置姿勢
    Vector4d qua;//クオータニオン
    Vector3d pos;//3軸位置
    std::cout << "xyz is \t" << pos(0) << " , " << pos(1) << " , "<< pos(2) << std::endl;
    double zangle = 0.0d;//set rotation
    angle(0) = -0.25d*M_PI;
    angle(1) = -0.25d*M_PI;
    angle(2) = -0.25d*M_PI;
    angle(3) = -0.25d*M_PI;
    angle(4) = -0.25d*M_PI;
    angle(5) = -0.25d*M_PI;
    angle(6) = -0.25d*M_PI;

    maninvk->calcaA(angle,mattheta);
    pos = mattheta.block(0,3,3,1);
    pos(0) -= 0.00001;
    qua = maninvk->matrixtoquatanion(mattheta);//回転行列からクオータニオンへ変換
    targetx.block(0,0,3,1) = pos;
    targetx.block(3,0,4,1) = qua.block(0,0,4,1);
    maninvk->settargetfx(targetx);
    maninvk->steepsetdescent(angle);
    exit(0);
    angle = maninvk->getangle(angle);
    std::cout << "angles are \t";
    for(int ii=0;ii<maninvk->getjointnum()-1;ii++){
        std::cout << angle(ii) << " , ";
    }
    std::cout << angle(maninvk->getjointnum()-1) <<  std::endl;
    Matrix4d ansmat;
    //PRINT_MAT(maninvk->penaltyfunc(angle));
    maninvk->calcaA(angle,ansmat);
    std::cout << "answer x: " << ansmat(0,3) << " , y: " << ansmat(1,3) << " , z: "<< ansmat(2,3) << std::endl;
}

int main(){
    int ii,jointn = 7;
    invkSolvenu *maninvk;
    maninvk = new invkSolvenu(jointn);
    /*RT CRANE*/
    maninvk->setdhparameter(0,0.0d*M_PI,0.0d,0.064d,-0.5d*M_PI);//(int num,double thoff,double aa,double di,double alph);
    maninvk->setdhparameter(1,0.0d*M_PI,0.0d,0.0d,0.5d*M_PI);//(int num,double thoff,double aa,double di,double alph);
    maninvk->setdhparameter(2,0.0d*M_PI,0.0d,0.065d+0.185d,-0.5d*M_PI);//(int num,double thoff,double aa,double di,double alph);
    maninvk->setdhparameter(3,0.0d*M_PI,0.0d,0.0d,0.5d*M_PI);//(int num,double thoff,double aa,double di,double alph);
    maninvk->setdhparameter(4,0.0d*M_PI,0.0d,0.121d+0.129d,-0.5d*M_PI);//(int num,double thoff,double aa,double di,double alph);
    maninvk->setdhparameter(5,0.0d*M_PI,0.0d,0.0d,0.5d*M_PI);//(int num,double thoff,double aa,double di,double alph);
    maninvk->setdhparameter(6,0.0d*M_PI,0.0d,0.019d+0.084d,0.0d);//(int num,double thoff,double aa,double di,double alph);
    //limit add
    VectorXd uplimit(7);
    VectorXd lowlimit(7);
    uplimit <<    2.72271 ,  0.5*M_PI  ,   2.72271  ,        0 ,   2.72271 ,    0.5*M_PI ,   2.89725;
    lowlimit <<  -2.72271 , -0.5*M_PI  ,  -2.72271  , -2.79253 ,  -2.72271 ,   -0.5*M_PI ,  -2.89725;
    maninvk->setlimit(uplimit,lowlimit);
    /**/
    VectorXd angle = VectorXd::Zero(jointn);
    Matrix4d mattheta = MatrixXd::Identity(4,4);//回転変位行列
    /*test*/
    inverse_kinematics(maninvk,angle,mattheta);//calc IK test
    forward_kinematics(maninvk,angle,mattheta);//calc FK test
    /**/
    delete maninvk;
    return 0;
}
#endif