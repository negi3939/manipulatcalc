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

#include "Negi39IKID.h"


void DHparameter::set(double l_thetaoffset,double l_a,double l_d,double l_alpha){
    thetaoffset = l_thetaoffset;
    a = l_a;
    d = l_d;
    alpha = l_alpha;
}

double DHparameter::getthetaoffset(){return thetaoffset;}
double DHparameter::geta(){return a;}
double DHparameter::getd(){return d;}
double DHparameter::getalpha(){return alpha;}

Negi39IKID::Negi39IKID(){}

Negi39IKID::Negi39IKID(std::vector<DHparameter> l_dh){
    dh = l_dh;
    limitfl = LIMITOFF;
    init();
}

Negi39IKID::Negi39IKID(std::vector<DHparameter> l_dh,VectorXd l_uplimit,VectorXd l_lowlimit){
    dh = l_dh;
    uplimit = l_uplimit;
    lowlimit = l_lowlimit;
    limitfl = LIMITON;
    init();
}

void Negi39IKID::init(){
    jointnum = dh.size();
    maninvk = new invkSolvenu(jointnum);
    maninvd = new invdSolvenu(jointnum);
    for(int ii=0;ii<dh.size();ii++){
        maninvk->setdhparameter(ii,dh[ii].getthetaoffset(),dh[ii].geta(), dh[ii].getd(),dh[ii].getalpha());
    }
    maninvd->copy(maninvk);
    //limit add
    if(limitfl!=LIMITOFF){
        maninvk->setlimit(uplimit,lowlimit);//可動範囲を設定（FLAGが立つ）
    }
    maninvk->setthreshold(0.00001d);//solverの判定thresholdを設定
    //init angle
    targetx = VectorXd::Zero(7);targetx(3) = 1.0d;
    angle_rad = VectorXd::Zero(6);
    angle_deg = VectorXd::Zero(6);
    angle_defo = VectorXd::Zero(6);
    setdefoko();// defoko pose
}

void Negi39IKID::setdefoko(VectorXd defo,ANGLEUNIT unit){
    switch (unit){
    case DEG:
        angle_deg = defo;
        angle_rad = degtorad(angle_deg);
        angle_defo = angle_rad;
        break;
    case RAD:
        angle_rad = defo;
        angle_deg = radtodeg(angle_rad);
        angle_defo = angle_rad;
        break;
    default:
        break;
    }
    setdefoko();
}

void Negi39IKID::setdefoko(){
    angle_rad = angle_defo;
    angle_deg = radtodeg(angle_defo);
    maninvk->calcaA(angle_rad,mattheta);
    PRINT_MAT(mattheta);
    pos = mattheta.block(0,3,3,1);
    qua = maninvk->matrixtoquatanion(mattheta);
    targetx.block(0,0,3,1) = pos;
    targetx.block(3,0,4,1) = qua.block(0,0,4,1);
    std::cout << " target is  "<< std::flush;
    for(int ii=0;ii<6;ii++){
        std::cout << targetx(ii) << " , " << std::flush;
    }
    std::cout << targetx(6) << std::endl;
}

VectorXd Negi39IKID::degtorad(VectorXd deg){return M_PI/180.0d*deg;}
VectorXd Negi39IKID::radtodeg(VectorXd rad){return 180.0d/M_PI*rad;}

VectorXd Negi39IKID::solve_relative(Vector3d target,EULERFL eulfl,double eulangle,ANGLEUNIT unit){
    Matrix3d rotateR;
    rotateR = Matrix3d::Identity();
    for(int ii=0;ii<3;ii++){
        targetx(ii) += target(ii);     
    }
    switch (eulfl){
    case NON:
        break;    
    case ROLL:
        rotateR(1,1) = cos(eulangle);
        rotateR(1,2) = -sin(eulangle);
        rotateR(2,2) = cos(eulangle);
        rotateR(2,1) = sin(eulangle);
        break;
    case PITCH:
        rotateR(0,0) = cos(eulangle);
        rotateR(0,2) = sin(eulangle);
        rotateR(2,0) = -sin(eulangle);
        rotateR(2,2) = cos(eulangle);
        break;
    case YAW://どういうわけかYAWとROLLが入れ替わってる
        rotateR(0,0) = cos(eulangle);
        rotateR(0,1) = -sin(eulangle);
        rotateR(1,1) = cos(eulangle);
        rotateR(1,0) = sin(eulangle);
        break;
    default:
        break;
    }
    if(eulfl!=NON){
        mattheta.block(0,0,3,3) = rotateR*mattheta.block(0,0,3,3);
    }
    pos = targetx.block(0,0,3,1);
    qua = maninvk->matrixtoquatanion(mattheta);
    targetx.block(3,0,4,1) = qua.block(0,0,4,1);
    PRINT_MAT(mattheta);
    //targetx(1) = 0.0d;
    maninvk->settargetfx(targetx);
    angle_rad = maninvk->getangle(angle_rad,NEWTON);
    angle_deg = radtodeg(angle_rad);
    if(unit==DEG){return angle_deg;}
    return angle_rad; 
}

VectorXd Negi39IKID::solve_abstarg(VectorXd target,ANGLEUNIT unit){
    for(int ii=0;ii<target.size();ii++){
        if(ii!=1){
        targetx(ii) = target(ii);
        }else{     
        }
    }
    pos = targetx.block(0,0,3,1);
    qua.block(0,0,4,1) = targetx.block(3,0,4,1);
    PRINT_MAT(mattheta);
    targetx(1) = 0.0d;
    maninvk->settargetfx(targetx);
    angle_rad = maninvk->getangle(angle_rad,NEWTON);
    angle_deg = radtodeg(angle_rad);
    if(unit==DEG){return angle_deg;}
    return angle_rad;
}

void Negi39IKID::show_angle(ANGLEUNIT unit=DEG){
    VectorXd l_angle;
    switch (unit){
    case DEG:
        l_angle = angle_deg;
        break;
    case RAD:
        l_angle = angle_rad;
        break;
    default:
        break;
    }
    std::cout << "angles are \t";
    for(int ii=0;ii<maninvk->getjointnum()-1;ii++){
        std::cout << l_angle(ii) << " , ";
    }
    std::cout << l_angle(maninvk->getjointnum()-1) <<  std::endl;
}

VectorXd Negi39IKID::setangle(VectorXd ang){
    angle_deg = ang;
    angle_rad = degtorad(angle_deg);
    maninvk->calcaA(angle_rad,mattheta);
    PRINT_MAT(mattheta);
    pos = mattheta.block(0,3,3,1);
    qua = maninvk->matrixtoquatanion(mattheta);
    targetx.block(0,0,3,1) = pos;
    targetx.block(3,0,4,1) = qua.block(0,0,4,1);
    return targetx;
}

#if defined(NEGI_IS_MAIN)
int main(){

    /*maninvk->setdhparameter(0,                            0.0d*M_PI,                                   0.055d, 0.21d,   -0.5d*M_PI);//(int num,double thoff,double aa,double di,double alph);
    maninvk->setdhparameter(1, -0.5d*M_PI+std::atan2(0.080d,0.420d), std::sqrt(0.420d*0.420d + 0.080d*0.080d),  0.0d,   0.0d*M_PI);//(int num,double thoff,double aa,double di,double alph);
    maninvk->setdhparameter(2,  0.5d*M_PI-std::atan2(0.080d,0.420d),                                     0.0d,  0.0d,  0.5d*M_PI);//(int num,double thoff,double aa,double di,double alph);
    maninvk->setdhparameter(3,                            0.0d*M_PI,                                     0.0d,  0.390d,  -0.5d*M_PI);//(int num,double thoff,double aa,double di,double alph);
    maninvk->setdhparameter(4,                            0.0d*M_PI,                                     0.0d,  0.0d,   0.5d*M_PI);//(int num,double thoff,double aa,double di,double alph);
    maninvk->setdhparameter(5,                            0.0d*M_PI,                                     0.0d,  0.045d + attachdis,   0.0d*M_PI);//(int num,double thoff,double aa,double di,double alph);
*/
    int jointnum = 6;
    std::vector<DHparameter> kardh(jointnum);
    double attachdis = 0.102d;
    kardh[0].set(                           0.0d*M_PI,                                   0.055d,                0.21d,  -0.5d*M_PI); 
    kardh[1].set(-0.5d*M_PI+std::atan2(0.080d,0.420d), std::sqrt(0.420d*0.420d + 0.080d*0.080d),                 0.0d,   0.0d*M_PI);
    kardh[2].set( 0.5d*M_PI-std::atan2(0.080d,0.420d),                                     0.0d,                 0.0d,   0.5d*M_PI);
    kardh[3].set(                           0.0d*M_PI,                                     0.0d,               0.390d,  -0.5d*M_PI);
    kardh[4].set(                           0.0d*M_PI,                                     0.0d,                 0.0d,   0.5d*M_PI);
    kardh[5].set(                           0.0d*M_PI,                                     0.0d,   0.045d + attachdis,   0.0d*M_PI);
    VectorXd uplimit(6);
    VectorXd lowlimit(6);
    uplimit <<    120.0d/180.0d*M_PI ,  70.0d/180.0d*M_PI  , 170.0d/180.0d*M_PI  ,  110.0d/180.0d*M_PI ,   75.0d/180.0d*M_PI ,    120.0d/180.0d*M_PI;//可動上限範囲を設定(1~6軸)
    lowlimit <<  -120.0d/180.0d*M_PI , -35.0d/180.0d*M_PI  ,  -0.0d/180.0d*M_PI  , -110.0d/180.0d*M_PI ,  -90.0d/180.0d*M_PI ,   -120.0d/180.0d*M_PI;//可動下限範囲を設定(1~6軸)
    
    Negi39IKID *ksik = new Negi39IKID(kardh,uplimit,lowlimit);
    VectorXd ang(jointnum);
    VectorXd angle_deg_defo(6);
    angle_deg_defo <<-18.6979 , 5.02352 , 98.8579 , -0.251937 , 75.2884 , 20.6719;
    ksik->setdefoko(angle_deg_defo,DEG);

    Vector3d pos;
    pos << -0.05,0.0,-0.0; 
    double eul = -60.0d*M_PI/180.0d;
    ksik->solve_relative(pos,ROLL,eul);
    ksik->show_angle();
    delete ksik;
}
#endif