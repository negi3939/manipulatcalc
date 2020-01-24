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
    std::stringstream filename;
    filename << "data/angleserial.dat";
    std::string f_n = filename.str();
	std::fstream fs;
    fs.open(f_n.c_str(),std::ios::out);
	if(! fs.is_open()) {
		std::cout << "=======cannot open file=========="<<std::endl;
		exit(1);
	}
    int ii,jointn = 6;
    double time = 0.0;
    double xpos = -0.2;
    invkSolvenu maninvk(jointn);
    invdSolvenu maninvd(jointn);
    maninvk.settime(time);
    maninvk.setdhparameter(0,M_PI,0.0,0.1519,M_PI/2.0);
    maninvk.setdhparameter(1,0.0,-0.24365,0.0,0.0);
    maninvk.setdhparameter(2,0.0,-0.21325,0.0,0.0);
    maninvk.setdhparameter(3,0.0,0.0,0.11235,M_PI/2.0);
    maninvk.setdhparameter(4,0.0,0.0,0.08535,-M_PI/2.0);
    maninvk.setdhparameter(5,0.0,0.0,0.0819,0.0);
    maninvd.copy(maninvk);
    double theta;
    Vector4d qua;
    Vector3d pos;
    VectorXd targetx(6);
    VectorXd targetvel(6);
    VectorXd angle =VectorXd::Zero(jointn);
    VectorXd angvel =VectorXd::Zero(jointn);
    Matrix4d mattheta = Matrix4d::Identity(4,4);
    pos(0) = xpos;//-0.230;
    pos(1) = 0.2;//-0.300;
    pos(2) = 0.3;//-0.400;
    while(1){//xpos<0.2){
        xpos = 0.04d*((double)time) -0.2;
        pos(0) = xpos;
        theta = 4.0*M_PI/3.0;
        mattheta(0,0) = cos(theta);
        mattheta(0,2) = sin(theta);
        mattheta(2,0) = -sin(theta);
        mattheta(2,2) = cos(theta);
        qua = maninvk.matrixtoquatanion(mattheta);
        targetx.block(0,0,3,1) = pos;
        targetx.block(3,0,3,1) = qua.block(0,0,3,1); 
        targetvel << 0.04d,0.0d,0.0d,0.0d,0.0d,0.0d;
        maninvk.settargetfx(targetx);
        maninvd.settargetfx(targetvel);
        angle = maninvk.getangle(angle);
        maninvd.calcaA(angle);
        angvel = maninvd.getvel(angvel);
        /*
        std::cout << "angle is "<<std::endl;
        fs << time << ",";
        std::cout << time << ",";
        for(ii=0;ii<3;ii++){
            fs << pos(ii) << ",";
            std::cout << pos(ii) << ",";
        }
        for(ii=0;ii<4;ii++){
            fs << qua(ii) << ",";
            std::cout<< qua(ii) << ",";
        }
        for(ii=0;ii<jointn;ii++){
            fs <<  angle(ii) << ",";
            std::cout << angle(ii) << ",";
        }
        
        for(ii=0;ii<jointn-1;ii++){
            fs <<  angvel(ii) << ",";
            std::cout << angvel(ii) << ",";
        }
        std::cout <<  angvel(jointn-1) << std::endl;
        fs <<  angvel(jointn-1) << std::endl;
        */
       std::cout << "time is:"<< time << "  invk class time is " << maninvd.gettime() << std::endl;
        time += 0.1;
    }
    fs.close();

    return 0;
}