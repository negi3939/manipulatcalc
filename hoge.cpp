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

double f(double x){
    return x * x * x - 2;
}
double g(double x){
    double ans,bef,aft;
    double deltax = 0.00000001;
    bef = (f(x)-f(x-deltax))/deltax;
    aft = (f(x+deltax)-f(x))/deltax;
    ans = (bef+aft)/2.0;
    if(ans==0){return 1.0;}
    return ans;  
}

int main(){
    double x = 0.0;
    while (1){
        double dx = x;
        x = x - (f(x) / g(x));
        printf("%12.10f %12.10f\n",dx,x);
        if (fabs(x - dx) < 0.0000000001) break;
    }

    printf("%12.10f %12.10f\n", x, cbrt(2));
    return 0;
}