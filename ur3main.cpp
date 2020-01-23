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

#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl
using namespace Mymath;

class invdSolvenu : public invkSolvenu {
  protected:
    Matrix4d *aTt;
    Vector3d *rra;
    Vector3d *ppa;
    Vector3d *zz;
    MatrixXd *jacobi;
  public:
    invdSolvenu();
    invdSolvenu(int num);
    VectorXd funcorg(VectorXd x) override;
    VectorXd getvel(VectorXd x);
    ~invdSolvenu();
};

invdSolvenu::invdSolvenu():invkSolvenu(){
    aTt = new Matrix4d[jointnum];
    rra = new Vector3d[jointnum];
    ppa = new Vector3d[jointnum];
    zz = new Vector3d[jointnum];
    jacobi = new MatrixXd;
    *jacobi= MatrixXd::Zero(6,jointnum);
    rra[0] = Vector3d::Zero(3,1);
    zz[0] << 0.0d,0.0d,1.0d;
}
invdSolvenu::invdSolvenu(int num):invkSolvenu(num){
    aTt = new Matrix4d[jointnum];
    rra = new Vector3d[jointnum];
    ppa = new Vector3d[jointnum];
    zz = new Vector3d[jointnum];
    jacobi = new MatrixXd;
    *jacobi = MatrixXd::Zero(6,jointnum);
    rra[0] = Vector3d::Zero(3,1);
    zz[0] << 0.0d,0.0d,1.0d;
}
invdSolvenu::~invdSolvenu(){delete jacobi;delete[] ppa;delete[] rra;delete[] aTt;delete[] zz;}
VectorXd invdSolvenu::funcorg(VectorXd x){
    int ii;
    VectorXd ans = VectorXd::Zero(6);
    Vector3d rrend;
    Matrix4d buff = aA[0];
    aTt[0] = buff;
    for(ii=1;ii<jointnum;ii++){
        buff = buff*aA[ii];
        aTt[ii] = buff;
    }
    rrend = aTt[jointnum-1].block(0,3,3,1);
    ppa[0] = rrend;
    jacobi->block(0,0,3,1) = zz[0].cross(ppa[0]);
    jacobi->block(3,0,3,1) = ppa[0];
    for(ii=1;ii<jointnum;ii++){
        rra[ii] = aTt[ii-1].block(0,3,3,1);
        zz[ii] = aTt[ii-1].block(0,2,3,1);
        ppa[ii] = rrend - rra[ii];
        jacobi->block(0,ii,3,1) = zz[ii].cross(ppa[ii]);
        jacobi->block(3,ii,3,1) = ppa[ii];
    }
    //PRINT_MAT(*jacobi);
    ans = (*jacobi)*x;
    PRINT_MAT(ans);
    
    return ans;
}

VectorXd invdSolvenu::getvel(VectorXd x){
    VectorXd vel = solve(x);
    return vel;
}

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
    pos(0) =xpos;//-0.230;
    pos(1) = 0.2;//-0.300;
    pos(2) = 0.3;//-0.400;
    while(xpos<0.2){
        xpos = 0.04d*((double)time) -0.2;
        PRINT_MAT(xpos);
        pos(0) = xpos;
        theta = 4.0*M_PI/3.0;
        mattheta(0,0) = cos(theta);
        mattheta(0,2) = sin(theta);
        mattheta(2,0) = -sin(theta);
        mattheta(2,2) = cos(theta);
        qua = maninvk.matrixtoquatanion(mattheta);
        targetx.block(0,0,3,1) = pos;
        targetx.block(3,0,3,1) = qua.block(0,0,3,1); 
        targetvel << 0.0,0.0,0.0,0.0,0.0,0.0;
        maninvk.settargetfx(targetx);
        angle = maninvk.getangle(angle);
        maninvd.calcaA(angle);
        angvel = maninvd.getvel(targetvel);
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
        */
        //std::cout <<  angvel(jointn-1) << std::endl;
        //fs <<  angvel(jointn-1) << std::endl;
        time += 0.1;
    }
    fs.close();

    return 0;
}