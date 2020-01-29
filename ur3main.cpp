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
void inputfvel(int ac,char *av[]);
void sankakuvel(); 

int main(int ac,char *av[]){
    if(ac>1){
        inputfvel(ac,av); 
    }else{
        sankakuvel();
    }
    return 0;
}

void inputfvel(int ac,char *av[]){
    std::ifstream ifs(av[1]);
    if (!ifs){
        std::cout << "cannot read file" << std::endl; //エラー
    }
    std::stringstream filename;
    filename << "data/angleserial.dat";
    std::string f_n = filename.str();
	std::fstream fs;
    fs.open(f_n.c_str(),std::ios::out);
	if(! fs.is_open()) {
		std::cout << "=======cannot open file=========="<<std::endl;
		exit(1);
	}
    int ii,jj,jointn = 6;
    double ddtt = 0.008d;
    double time = 0.0d;
    double xinipos = -0.15d;
    double xpos = xinipos;
    double zinipos = 0.3d;
    double zpos = zinipos;
    double thetaini = 1.0d*M_PI;
    double theta = thetaini;
    double xvel = 0.0d;
    double zvel = 0.0d;
    invkSolvenu maninvk(jointn);
    invdSolvenu maninvd(jointn);
    maninvk.settime(time);
    maninvk.setdhparameter(0,M_PI,0.0d,0.1519d,M_PI/2.0d);
    maninvk.setdhparameter(1,0.0d,-0.24365d,0.0d,0.0d);
    maninvk.setdhparameter(2,0.0d,-0.21325d,0.0d,0.0d);
    maninvk.setdhparameter(3,0.0d,0.0d,0.11235d,M_PI/2.0d);
    maninvk.setdhparameter(4,0.0d,0.0d,0.08535d,-M_PI/2.0d);
    maninvk.setdhparameter(5,0.0d,0.0d,0.0819d +0.055d,0.0d);//手先補正(Festo)
    maninvd.copy(maninvk);
    Vector4d qua;
    Vector3d pos;
    VectorXd targetx(6);
    VectorXd targetvel(6);
    VectorXd angle =VectorXd::Zero(jointn);
    VectorXd preangle =VectorXd::Zero(jointn);
    VectorXd anglewrite =VectorXd::Zero(jointn);
    VectorXd angledoublepi =VectorXd::Zero(jointn);
    VectorXd angvel =VectorXd::Zero(jointn);
    Matrix4d mattheta = Matrix4d::Identity(4,4);
    pos(0) = xpos;//-0.230;
    pos(1) = 0.4d;//-0.300;
    pos(2) = zpos;//-0.400;
    std::vector<double> timef,xf,zf,thetaf,vxf,vzf,vthetaf;
    std::string line;
    for( int row = 0;getline(ifs, line);++row ) { // 1行読んで
        std::istringstream stream(line);
        double data;
        for(int col = 0; stream >> data; ++col) { // 1個ずつ切り分ける
            switch (col){
                case 0:
                    timef.push_back(data);
                    break; 
                case 1:
                    xf.push_back(data);
                    break; 
                case 2:
                    zf.push_back(data);
                    break; 
                case 3:
                    thetaf.push_back(data);
                    break; 
                case 4:
                    vxf.push_back(data);
                    break;
                case 5:
                    vzf.push_back(data);
                    break; 
                case 6:
                    vthetaf.push_back(data);
                    break; 
                default:
                    break;
            }
        }
    }
    double ddttf = timef[1] -timef[0];
    for(ii=0;ii<timef.size();ii++){
        if((timef[ii]-((int)(timef[ii]/ddtt))*ddtt) < (ddttf/2.0d)){
            time = ((int)(timef[ii]/ddtt))*ddtt;
            xpos = xinipos + xf[ii];
            zpos = zinipos + zf[ii];
            theta = thetaini + thetaf[ii];
            pos(0) = xpos;
            pos(2) = zpos;
            mattheta(0,0) = cos(theta);
            mattheta(0,2) = sin(theta);
            mattheta(2,0) = -sin(theta);
            mattheta(2,2) = cos(theta);
            qua = maninvk.matrixtoquatanion(mattheta);
            targetx.block(0,0,3,1) = pos;
            targetx.block(3,0,3,1) = qua.block(0,0,3,1); 
            targetvel << vxf[ii],0.0d,vzf[ii],0.0d,vthetaf[ii],0.0d;
            maninvk.settargetfx(targetx);
            maninvd.settargetfx(targetvel);
            preangle = angle;
            angle = maninvk.getangle(angle);
            maninvd.calcaA(angle);
            angvel = maninvd.getvel(angvel);
            for(jj=0;jj<jointn;jj++){
                if(angle(jj)-preangle(jj)>1.5d*M_PI){
                    angledoublepi(jj) -= 1.0d;
                }else if(angle(jj)-preangle(jj)<-1.5d*M_PI){
                    angledoublepi(jj) += 1.0d;
                }
                anglewrite(jj) = angle(jj) + 2.0d*M_PI*angledoublepi(jj);
            }
            std::cout << "angle is "<<std::endl;
            fs << time << ",";
            std::cout << time << ",";
            for(jj=0;jj<3;jj++){
                fs << pos(jj) << ",";
                std::cout << pos(jj) << ",";
            }
            for(jj=0;jj<4;jj++){
                fs << qua(jj) << ",";
                std::cout<< qua(jj) << ",";
            }
            for(jj=0;jj<jointn;jj++){
                fs <<  anglewrite(jj) << ",";
                std::cout << anglewrite(jj) << ",";
            }
        
            for(jj=0;jj<jointn-1;jj++){
                fs <<  angvel(jj) << ",";
                std::cout << angvel(jj) << ",";
            }
            std::cout <<  angvel(jointn-1) << std::endl;
            fs <<  angvel(jointn-1) << std::endl;
        }

    }
    fs.close();
}

void sankakuvel(){
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
    double ddtt = 0.008d;
    double time = 0.0d;
    double timecha = 0.0d;
    double xpos = -0.15d;
    double xvel = 0.0d;
    double xvelcha = 0.0d;
    invkSolvenu maninvk(jointn);
    invdSolvenu maninvd(jointn);
    maninvk.settime(time);
    maninvk.setdhparameter(0,M_PI,0.0d,0.1519d,M_PI/2.0d);
    maninvk.setdhparameter(1,0.0d,-0.24365d,0.0d,0.0d);
    maninvk.setdhparameter(2,0.0d,-0.21325d,0.0d,0.0d);
    maninvk.setdhparameter(3,0.0d,0.0d,0.11235d,M_PI/2.0d);
    maninvk.setdhparameter(4,0.0d,0.0d,0.08535d,-M_PI/2.0d);
    maninvk.setdhparameter(5,0.0d,0.0d,0.0819d +0.055d,0.0d);//手先補正(Festo)
    maninvd.copy(maninvk);
    double theta;
    Vector4d qua;
    Vector3d pos;
    VectorXd targetx(6);
    VectorXd targetvel(6);
    VectorXd angle =VectorXd::Zero(jointn);
    VectorXd preangle =VectorXd::Zero(jointn);
    VectorXd anglewrite =VectorXd::Zero(jointn);
    VectorXd angledoublepi =VectorXd::Zero(jointn);
    VectorXd angvel =VectorXd::Zero(jointn);
    Matrix4d mattheta = Matrix4d::Identity(4,4);
    pos(0) = xpos;//-0.230;
    pos(1) = 0.4d;//-0.300;
    pos(2) = 0.3d;//-0.400;
    
    double acc = 7.0d;
    while(xpos<0.15){
        if(xpos<0.0d){
            xvel = xvel+acc*ddtt;
            timecha = time;
            xvelcha = xvel;
        }else{
            xvel = xvel-acc*ddtt;
            if(xvel<0.0d){break;}
        }
        xpos = xpos + xvel*ddtt;
        pos(0) = xpos;
        theta = 2.0*M_PI/2.0d;
        mattheta(0,0) = cos(theta);
        mattheta(0,2) = sin(theta);
        mattheta(2,0) = -sin(theta);
        mattheta(2,2) = cos(theta);
        qua = maninvk.matrixtoquatanion(mattheta);
        targetx.block(0,0,3,1) = pos;
        targetx.block(3,0,3,1) = qua.block(0,0,3,1); 
        targetvel << xvel,0.0d,0.0d,0.0d,0.0d,0.0d;
        maninvk.settargetfx(targetx);
        maninvd.settargetfx(targetvel);
        preangle = angle;
        angle = maninvk.getangle(angle);
        maninvd.calcaA(angle);
        angvel = maninvd.getvel(angvel);
        
        for(ii=0;ii<jointn;ii++){
            if(angle(ii)-preangle(ii)>1.5d*M_PI){
                angledoublepi(ii) -= 1.0d;
            }else if(angle(ii)-preangle(ii)<-1.5d*M_PI){
                angledoublepi(ii) += 1.0d;
            }
            anglewrite(ii) = angle(ii) + 2.0d*M_PI*angledoublepi(ii);
        }
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
            fs <<  anglewrite(ii) << ",";
            std::cout << anglewrite(ii) << ",";
        }
        
        for(ii=0;ii<jointn-1;ii++){
            fs <<  angvel(ii) << ",";
            std::cout << angvel(ii) << ",";
        }
        std::cout <<  angvel(jointn-1) << std::endl;
        fs <<  angvel(jointn-1) << std::endl;
        time += ddtt;
    }
    fs.close();
}