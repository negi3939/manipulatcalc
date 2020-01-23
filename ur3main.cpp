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

#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl
using namespace Mymath;


int main(){
    int ii,jointn = 6;
    maniSolvenu mani(jointn);
    mani.setdhparameter(0,M_PI,0.0,0.1519,M_PI/2.0);
    mani.setdhparameter(1,0.0,-0.24365,0.0,0.0);
    mani.setdhparameter(2,0.0,-0.21325,0.0,0.0);
    mani.setdhparameter(3,0.0,0.0,0.11235,M_PI/2.0);
    mani.setdhparameter(4,0.0,0.0,0.08535,-M_PI/2.0);
    mani.setdhparameter(5,0.0,0.0,0.0819,0.0);
    double theta;
    Vector4d qua;
    Vector3d pos;
    VectorXd targetx(6);
    VectorXd angle(jointn);
    Matrix4d mattheta = Matrix4d::Identity(4,4);
    pos(0) = 0.1;//-0.230;
    pos(1) = 0.2;//-0.300;
    pos(2) = 0.3;//-0.400;
    theta = 4.0*M_PI/3.0;
    mattheta(0,0) = cos(theta);
    mattheta(0,2) = sin(theta);
    mattheta(2,0) = -sin(theta);
    mattheta(2,2) = cos(theta);
    qua = mani.matrixtoquatanion(mattheta);
    targetx.block(0,0,3,1) = pos;
    targetx.block(3,0,3,1) = qua.block(0,0,3,1); 
    mani.settargetfx(targetx);
    angle = mani.getangle(angle);
    std::cout << "angle is "<<std::endl;
    for(ii=0;ii<jointn;ii++){
        std::cout << angle(ii) << ",";
    }
    std::cout << std::endl;

    return 0;
}