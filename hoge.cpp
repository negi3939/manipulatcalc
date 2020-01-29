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

#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl
using namespace Mymath;

int main(int ac,char *av[]){
    std::ifstream ifs(av[1]);
    std::vector<double> timef,xf,yf,thetaf,vxf,vyf,vthetaf;
    if (!ifs){
        std::cout << "cannot read file" << std::endl; //エラー
    }
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
                    yf.push_back(data);
                    break; 
                case 3:
                    thetaf.push_back(data);
                    break; 
                case 4:
                    vxf.push_back(data);
                    break;
                case 5:
                    vyf.push_back(data);
                    break; 
                case 6:
                    vthetaf.push_back(data);
                    break; 
                default:
                    break;
            }
        }
    }
    for(int ii=0;ii<timef.size();ii++){
        std::cout << " time is " << timef[ii] << " x is " << xf[ii] << " y is " << yf[ii] << " theta is " << thetaf[ii] << std::endl; 
    }
    std::cout <<" ac is "<< ac <<std::endl;
    return 0;
}