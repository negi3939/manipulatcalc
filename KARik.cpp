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
#include "inversedynamics.h"

#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl
using namespace Mymath;



int main(){
    int ii,jointn = 6;
    Matrix4d mattheta;
    invkSolvenu *maninvk;
    maninvk = new invkSolvenu(jointn);
    /*KAR*/
    double attachdis = 0.102d;
    //                      jointnum             thetaoff                       aa                    di           alpha
    maninvk->setdhparameter(0,                       0.0d*M_PI,                              0.055d, 0.21d,   0.5d*M_PI);//(int num,double thoff,double aa,double di,double alph);
    maninvk->setdhparameter(1,0.5d*M_PI- atan2(0.080d,0.420d), sqrt(0.420d*0.420d + 0.080d*0.080d),  0.0d,   -1.0d*M_PI);//(int num,double thoff,double aa,double di,double alph);
    maninvk->setdhparameter(2,           -atan2(0.080d,0.420d),                             0.390d,  0.0d,   0.5d*M_PI);//(int num,double thoff,double aa,double di,double alph);
    maninvk->setdhparameter(3,                       0.0d*M_PI,                                0.0d,  0.0d,  -0.5d*M_PI);//(int num,double thoff,double aa,double di,double alph);
    maninvk->setdhparameter(4,                       0.0d*M_PI,                  0.045d + attachdis,  0.0d,   0.5d*M_PI);//(int num,double thoff,double aa,double di,double alph);
    maninvk->setdhparameter(5,                       0.0d*M_PI,                                0.0d,  0.0d,   0.0d*M_PI);//(int num,double thoff,double aa,double di,double alph);
    //limit add
    VectorXd angle = VectorXd::Zero(6);
    angle(2) = 0.5d*M_PI;
    maninvk->calcaA(angle,mattheta);
    PRINT_MAT(mattheta);
    std::cout << "atan2 val "<< (-0.5d*M_PI+ atan2(0.080d,0.420d))*180/M_PI << std::endl;
    VectorXd uplimit(6);
    VectorXd lowlimit(6);
    uplimit <<    2.72271 ,  0.5*M_PI  ,   2.72271  ,        0 ,   2.72271 ,    0.5*M_PI;//可動上限範囲を設定
    lowlimit <<  -2.72271 , -0.5*M_PI  ,  -2.72271  , -2.79253 ,  -2.72271 ,   -0.5*M_PI;//可動下限範囲を設定
    maninvk->setlimit(uplimit,lowlimit);//可動範囲を設定（FLAGが立つ）

}