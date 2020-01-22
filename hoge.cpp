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

#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl
using namespace Mymath;

class hogeSolvenu :public Solvenu{
    protected:
    public:
        VectorXd funcorg(VectorXd x) override;
};

VectorXd hogeSolvenu::funcorg(VectorXd x){
    VectorXd ans(2);
    MatrixXd aA(ans.size(),x.size());
    ans(0) = x(0)*x(0) + x(1)*x(1) + x(2);
    ans(1) = x(0) + x(1)*x(1) + x(2)*x(2);
    return ans;
}


class maniSolvenu : public Solvenu {
  protected:
    int jointnum;
    Matrix4d *aA;
    Matrix4d *aAdis;
    Matrix4d *aAtheta;
    Matrix4d *aAaal;
    Matrix4d *aAalp;
    double *aal;
    double *alp;
    double *dis;
    void init();
  public:
    maniSolvenu();
    maniSolvenu(int num);
    VectorXd funcorg(VectorXd x) override;
    void setdhparameter(int num,double aa,double di,double alph);
    Vector4d matrixtoquatanion(Matrix4d mat);
    ~maniSolvenu();
};

maniSolvenu::maniSolvenu(){jointnum = 6;init();}
maniSolvenu::maniSolvenu(int num){jointnum = num;init();}

void maniSolvenu::init(){
    aA = new Matrix4d[jointnum];
    aAdis = new Matrix4d[jointnum];
    aAtheta = new Matrix4d[jointnum];
    aAaal = new Matrix4d[jointnum];
    aAalp = new Matrix4d[jointnum];
    aal = new double[jointnum];
    alp = new double[jointnum];
    dis = new double[jointnum];
    for(int ii=0;ii<jointnum;ii++){
        aAdis[ii] = MatrixXd::Identity(4,4);
        aAtheta[ii] = MatrixXd::Identity(4,4);
        aAaal[ii] = MatrixXd::Identity(4,4);
        aAalp[ii] = MatrixXd::Identity(4,4);
    }
}

VectorXd maniSolvenu::funcorg(VectorXd x){
    VectorXd ans(6);
    MatrixXd mat = MatrixXd::Identity(6,6);
    ans = mat*x;
    return ans;
    /*
    int ii;
    Vector3d pos;
    Vector4d qua;
    VectorXd ans(6);
    for(ii=0;ii<jointnum;ii++){
        //x(ii) = x(ii) - sign(x(ii))*(double)((int)(x(ii)/2.0/M_PI));
        aAtheta[ii](0,0) = cos(x(ii));
        aAtheta[ii](0,1) = -sin(x(ii));
        aAtheta[ii](1,0) = sin(x(ii));
        aAtheta[ii](1,1) = cos(x(ii));
        aA[ii] = aAdis[ii]*aAtheta[ii]*aAaal[ii]*aAalp[ii];
    }
    
    Matrix4d allA = aA[0];
    for(ii=1;ii<jointnum;ii++){
        allA = allA*aA[ii];
    }
    
    qua = matrixtoquatanion(allA);
    pos = allA.block(0,3,3,1);
    ans.block(0,0,3,1) = pos;
    ans.block(3,0,3,1) = qua.block(0,0,3,1);
    return ans;*/
}

void maniSolvenu::setdhparameter(int num,double aa,double di,double alph){
    aal[num] = aa;
    dis[num] = di;
    alp[num] = alph;
    aAdis[num](2,3) = dis[num];
    aAaal[num](0,3) = aal[num];
    aAalp[num](1,1) = cos(alp[num]);
    aAalp[num](1,2) = -sin(alp[num]);
    aAalp[num](2,1) = sin(alp[num]);
    aAalp[num](2,2) = cos(alp[num]);
}

Vector4d maniSolvenu::matrixtoquatanion(Matrix4d mat){
    Vector4d ans;
    ans(3) = sqrt((1.0 + mat(0,0) + mat(1,1) + mat(2,2))/4.0);
    ans(0) = (mat(1,2) -mat(2,1))/4.0/ans(3);
    ans(1) = (mat(2,0) -mat(0,2))/4.0/ans(3);
    ans(2) = (mat(0,1) -mat(1,0))/4.0/ans(3);
    return ans;
}

maniSolvenu::~maniSolvenu(){
    delete[] aal;
    delete[] alp;
    delete[] dis;
    delete[] aAalp;
    delete[] aAaal;
    delete[] aAtheta;
    delete[] aAdis;
    delete[] aA;
}


int main(){
    int jointn = 6;
    maniSolvenu mani(jointn);
    mani.setdhparameter(0,0.0,0.1519,M_PI/2.0);
    mani.setdhparameter(1,-0.24365,0.0,0.0);
    mani.setdhparameter(2,-0.21325,0.0,0.0);
    mani.setdhparameter(3,0.0,0.11235,M_PI/2.0);
    mani.setdhparameter(4,0.0,0.08535,-M_PI/2.0);
    mani.setdhparameter(5,0.0,0.0819,0.0);
    double theta;
    Vector4d qua;
    Vector3d pos;
    VectorXd targetx(6);
    VectorXd angle(jointn);
    Matrix4d mattheta = Matrix4d::Identity(4,4);
    pos(0) = -0.230;
    pos(1) = -0.300;
    pos(2) = -0.400;
    theta = M_PI/6+M_PI;
    mattheta(0,0) = cos(theta);
    mattheta(0,2) = sin(theta);
    mattheta(2,0) = -sin(theta);
    mattheta(2,2) = cos(theta);
    qua = mani.matrixtoquatanion(mattheta);
    targetx.block(0,0,3,1) = pos;
    targetx.block(3,0,3,1) = qua.block(0,0,3,1); 
    mani.settargetfx(targetx);
    angle = mani.solve(angle);
    PRINT_MAT(angle);
    /*
    VectorXd x = VectorXd::Zero(3);
    VectorXd dx = x;
    VectorXd chk;
    VectorXd tarfx(2);
    tarfx << 8,14;
    hogeSolvenu eqex;
    eqex.settargetfx(tarfx);
    x = eqex.solve(x);
    PRINT_MAT(eqex.funcorg(x));
    std::cout << "success calculated" << std::endl;
    */
    return 0;
}