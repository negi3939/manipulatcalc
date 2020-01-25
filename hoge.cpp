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
#include "animat.h"
//#include"figure.h"

#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl
using namespace Mymath;

class Hoge{
    private:
        int a;
    protected:
        static void* staticviewa(void *pParam) {
        	std::cout << "in static space :" << reinterpret_cast<Hoge*>(pParam)->geta() << std::endl;
      	}
    public:
        void seta(int n);
        void viewa();
        int geta();
};

void Hoge::seta(int n){a =n;}
void Hoge::viewa(){staticviewa(this);}
int Hoge::geta(){return a;}


int main(int ac,char *av[]){
    //Animat anihoge;
    //initialGlut(ac,av);
    //while(1){}
    Hoge hoge;
    for(int ii=0;ii<100;ii++){
        hoge.seta(ii);
        hoge.viewa();
    }
    
    return 0;
}