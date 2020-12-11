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

Solvenu::Solvenu(){countlimit=1000;limitfl=LIMITOFF;threshold=0.000001d;}
Solvenu::~Solvenu(){}

void Solvenu::setcountlimit(long a){countlimit = a;}
void Solvenu::setthreshold(double l_threshold){threshold = l_threshold;}

void Solvenu::setlimit(VectorXd uplimit,VectorXd lowlimit){
    upperlimit = uplimit;
    lowerlimit = lowlimit;
    limitfl = LIMITON;
}

void Solvenu::unsetlimit(){
    limitfl = LIMITOFF;
}

int Solvenu::checklimit(VectorXd &x){
    if(limitfl==LIMITOFF){return 0;}
    int fl=0;
    for(int ii=0;ii<x.size();ii++){
        //errval(ii) = step(x(ii) - upperlimit(ii))*abs(x(ii) - upperlimit(ii)) - step( - x(ii) + lowerlimit(ii))*abs( - x(ii) + lowerlimit(ii)); 
        if(x(ii)> upperlimit(ii)){x(ii) = upperlimit(ii);fl++;}
        else if(x(ii)< lowerlimit(ii)){x(ii) = lowerlimit(ii);fl++;}
    }
    return fl;
}

VectorXd Solvenu::sigmoidlimit(VectorXd &x,double alpha){
    VectorXd ans = VectorXd::Zero(1);
    if(limitfl==LIMITOFF){return ans;}
    for(int ii=0;ii<x.size();ii++){
        ans(0) += sigmoid(x(ii)-upperlimit(ii),alpha);
    }
    return ans;
}

VectorXd Solvenu::penaltyfunc(VectorXd &x){
    VectorXd ans =VectorXd::Zero(x.size()) ;
    for(int ii=0;ii<x.size();ii++){
        ans(ii) += step(x(ii)-upperlimit(ii))*(x(ii)-upperlimit(ii))*(x(ii)-upperlimit(ii));
        ans(ii) += step(-x(ii)+lowerlimit(ii))*(-x(ii)+lowerlimit(ii))*(-x(ii)+lowerlimit(ii));
    }
    return ans;
}

void Solvenu::settargetfx(VectorXd tfx){targetfx = tfx;}
VectorXd Solvenu::gettargetfx(){return targetfx;}

VectorXd Solvenu::function(VectorXd x){
    return functionerror(x);
}

VectorXd Solvenu::funcorg(VectorXd x){
    VectorXd ans(2);
    MatrixXd aA(ans.size(),x.size());
    aA << 1,0,0,1;
    ans = aA*x;
    return ans;
}

VectorXd Solvenu::functionerror(VectorXd x){
    //checklimit(x);
    VectorXd buf = funcorg(x);
    VectorXd trgfx(buf.size());
    if(limitfl!=LIMITON){ 
        return  funcorg(x) - targetfx;
    }else{
        //PRINT_MAT(funcorg(x));
        //PRINT_MAT(targetfx);
        //PRINT_MAT(funcorg(x)- targetfx);
        MatrixXd oone = 100.0d*MatrixXd::Ones(targetfx.size(),x.size());
        return  funcorg(x) - targetfx + oone*penaltyfunc(x);
    }
}

VectorXd Solvenu::solve(VectorXd intx,SolvFLAG slflag=NEWTON,double l_alpha=0,JudgeFLAG jdgfl=ZERO){
    switch (slflag){
    case NEWTON:
        return newtonsolve(intx,jdgfl);
        break;
    case STEEPEST:
        if(l_alpha<=0){return steepsetdescentsolve(intx,jdgfl);}
        return steepsetdescentsolve(intx,l_alpha,jdgfl);
        break;
    default:
        return intx;
        break;
    }
}

VectorXd Solvenu::newtonsolve(VectorXd intx,JudgeFLAG jdgfl){
    x = intx;
    long count = 0;
    VectorXd dx = intx;
    MatrixXd buf;
    VectorXd chk;
    while(1){
        dx = x;
        JacobiSVD<MatrixXd> svd(diffvec(x,this), ComputeThinU|ComputeThinV);
        x = x - svd.solve(functionerror(x));//limit ここを書き換える必要がある
        //PRINT_MAT(x);
        chk = MatrixXd::Ones(1,x.size())*absmat(x - dx);// + sigmoidlimit(x,1000);
        if(count>countlimit){std::cout <<"CAUTIONCAUTIONCAUTIONCAUTIONCAUTION step is more than" << countlimit << "CAUTIONCAUTIONCAUTIONCAUTIONCAUTIONC"<<std::endl;PRINT_MAT(functionerror(x));break;}
        if ((jdgfl==ZERO)&&(functionerror(x).norm() < threshold)){ break;}
        else if((jdgfl==DIFFZERO)&&(diffnorm(x,this).norm()<threshold)){break;}
        count++;
    }
    int checkret=0;
    if(limitfl==LIMITCHECK){checkret = checklimit(x);}
    if(checkret){std::cout << "solve fail :" << checkret << std::endl;}
    return x;
}

VectorXd Solvenu::newtonsolve(VectorXd intx,MatrixXd &l_jacobi,JudgeFLAG jdgfl){
    x = intx;
    long count = 0;
    VectorXd dx = intx;
    MatrixXd buf;
    VectorXd chk;
    while(1){
        dx = x;
        JacobiSVD<MatrixXd> svd(l_jacobi, ComputeThinU|ComputeThinV);
        x = x - svd.solve(functionerror(x));//limit ここを書き換える必要がある
        //PRINT_MAT(x);
        chk = MatrixXd::Ones(1,x.size())*absmat(x - dx);// + sigmoidlimit(x,1000);
        if(count>countlimit){std::cout <<"CAUTIONCAUTIONCAUTIONCAUTIONCAUTION step is more than" << countlimit << "CAUTIONCAUTIONCAUTIONCAUTIONCAUTIONC"<<std::endl;PRINT_MAT(functionerror(x));break;}
        if ((jdgfl==ZERO)&&(functionerror(x).norm() < threshold)){ break;}
        else if((jdgfl==DIFFZERO)&&(diffnorm(x,this).norm()<threshold)){break;}
        count++;
    }
    int checkret=0;
    if(limitfl==LIMITCHECK){checkret = checklimit(x);}
    if(checkret){std::cout << "solve fail :" << checkret << std::endl;}
    return x;
}

VectorXd Solvenu::steepsetdescentsolve(VectorXd intx,JudgeFLAG jdgfl){
    long count = 0;
    VectorXd x = intx;
    MatrixXd diffv;
    double gold_r = 0.6180339887d;
    double bottom_alpha,top_alpha,alpha1,alpha2;
    MatrixXd s;
    while(1){
        diffv =  diffnorm(x,this);
        if((jdgfl==ZERO)&&(functionerror(x).norm()<threshold)){break;}
        else if((jdgfl==DIFFZERO)&&(diffnorm(x,this).norm()<threshold)){break;}
        //std::cout << "norm is " <<functionerror(x).norm() << std::endl;
        s = -diffv.transpose();
        bottom_alpha=0.0d;
        top_alpha=10.0d;
        alpha1 = bottom_alpha + (1.0d -gold_r)*(top_alpha - bottom_alpha);
        alpha2 = bottom_alpha + gold_r*(top_alpha - bottom_alpha);
        while(1){
            if(functionerror(x + alpha1*s).norm()<functionerror(x + alpha2*s).norm()){
                top_alpha = alpha2;
                alpha2 = alpha1;
                alpha1 = bottom_alpha + (1.0d -gold_r)*(top_alpha - bottom_alpha);
            }else{
                bottom_alpha = alpha1;
                alpha1 = alpha2;
                alpha2 = bottom_alpha + gold_r*(top_alpha - bottom_alpha);
            }
            if(count>countlimit){std::cout <<"CAUTIONCAUTIONCAUTIONCAUTIONCAUTION step is more than" << countlimit << "CAUTIONCAUTIONCAUTIONCAUTIONCAUTIONC"<<std::endl;PRINT_MAT(functionerror(x));break;}
            //std::cout << "bottom_alpha : "<< bottom_alpha << " top_alpha : "<< top_alpha << std::endl;
            if(std::abs(bottom_alpha - top_alpha) <threshold){break;}
        }
        x = x + 0.5d*(bottom_alpha+top_alpha)*s;
    }
    int checkret=0;
    if(limitfl==LIMITCHECK){checkret = checklimit(x);}
    if(checkret){std::cout << "solve fail :" << checkret << std::endl;}
    return x;
}

VectorXd Solvenu::steepsetdescentsolve(VectorXd intx,double l_alpha,JudgeFLAG jdgfl){
    long count = 0;
    VectorXd x = intx;
    MatrixXd diffv;
    MatrixXd s;
    while(1){
        //diffv =  diffnorm(x,this);
        if((jdgfl==ZERO)&&(functionerror(x).norm()<threshold)){break;}
        else if((jdgfl==DIFFZERO)&&(diffnorm(x,this).norm() < threshold)){break;}

        //std::cout << "alppha is const. norm is " <<functionerror(x).norm() << std::endl;
        s = -diffv.transpose();
        x = x + l_alpha*s;
    }
    int checkret=0;
    if(limitfl==LIMITCHECK){checkret = checklimit(x);}
    if(checkret){std::cout << "solve fail :" << checkret << std::endl;}
    return x;
}
 
class SolveSQP{
    protected:
        MatrixXd Hk;
        SolveNU::Functype soltype;
        int xvecsize;
        double alpha;
        double threshold;
        Funcvec *optfunc;
        Funcvec *constrainth;
        Funcvec *constraintg;
        Funcvec *constrainthg;
        static VectorXd solve_noconstraints_st(VectorXd &x,SolveSQP *p){
             return p->solve_noconstraints(x);
        }
        static VectorXd solve_eqconstraints_st(VectorXd &x,SolveSQP *p){
            return p->solve_eqconstraints(x);
        }
        static VectorXd solve_ineqconstraints_st(VectorXd &x,SolveSQP *p){
            return p->solve_ineqconstraints(x);
        }
        static VectorXd solve_bothconstraints_st(VectorXd &x,SolveSQP *p){
            return p->solve_bothconstraints(x);
        }
        VectorXd solve_noconstraints(VectorXd &x);
        VectorXd solve_eqconstraints(VectorXd &x);
        VectorXd solve_ineqconstraints(VectorXd &x);
        VectorXd solve_bothconstraints(VectorXd &x);
        VectorXd (*solvefuncp)(VectorXd &x,SolveSQP *p);
    public:
        SolveSQP(Funcvec *l_optfunc);
        SolveSQP(Funcvec *l_optfunc,Funcvec *l_constraint,SolveNU::Functype slf);
        SolveSQP(Funcvec *l_optfunc,Funcvec *l_constrainth,Funcvec *l_constraintg);
        void init();
        double calcoptfunc(VectorXd &x);
        VectorXd constraintineqg(VectorXd &x);
        VectorXd constrainteqh(VectorXd &x);
        VectorXd calcdeltax_sf(VectorXd &x);
        VectorXd calcdeltax_sf(VectorXd &x,VectorXd &s_rho);
        VectorXd solve(VectorXd x);
};

SolveSQP::SolveSQP(Funcvec *l_optfunc){
    optfunc = l_optfunc;
    soltype = SolveNU::OPTFUNC;
    init();
}

SolveSQP::SolveSQP(Funcvec *l_optfunc,Funcvec *l_constraint,SolveNU::Functype slf){
    optfunc = l_optfunc;
    switch(slf){
        case SolveNU::INEQCONSTRAINT :
            constraintg = l_constraint;
            soltype = slf;
            break;
        case SolveNU::EQCONSTRAINT :
            constrainth = l_constraint;
            soltype = slf;
            break;
        default:
            break;
    }
    init();
}

SolveSQP::SolveSQP(Funcvec *l_optfunc,Funcvec *l_constrainth,Funcvec *l_constraintg){
    optfunc = l_optfunc;
    constrainth = l_constrainth;
    constraintg = l_constraintg;
    soltype = SolveNU::BOTHSTRAINT;
    init();
}

void SolveSQP::init(){
    xvecsize  =  optfunc->getxvecsize();
    Hk = MatrixXd::Identity(xvecsize,xvecsize);
    switch(soltype){
        case SolveNU::OPTFUNC :
            solvefuncp = SolveSQP::solve_noconstraints_st;
            break;
        case SolveNU::EQCONSTRAINT :
            solvefuncp = SolveSQP::solve_eqconstraints_st;
            break;
        case SolveNU::INEQCONSTRAINT :
            solvefuncp = SolveSQP::solve_ineqconstraints_st;
            break;
        case SolveNU::BOTHSTRAINT :
            solvefuncp = SolveSQP::solve_bothconstraints_st;
            break;
        default :
            break;

    }
    alpha = 0.1d;
    threshold = 0.00000000001;
}

double SolveSQP::calcoptfunc(VectorXd &x){
    return optfunc->function(x)(0);
}

VectorXd SolveSQP::constraintineqg(VectorXd &x){
    return constraintg->function(x);
}

VectorXd SolveSQP::constrainteqh(VectorXd &x){
    return constrainth->function(x);
}

VectorXd SolveSQP::calcdeltax_sf(VectorXd &x,VectorXd &s_rho){
    VectorXd ans;
    ans = -(diffvec(x,optfunc).transpose() + Hk*s_rho);
    return ans;
}

VectorXd SolveSQP::calcdeltax_sf(VectorXd &x){
    VectorXd ans;
    ans = -diffvec(x,optfunc).transpose();
    return ans;
}

VectorXd SolveSQP::solve(VectorXd x){
    return solvefuncp(x,this);
}

VectorXd SolveSQP::solve_noconstraints(VectorXd &x){
    VectorXd ans;
    std::cout << "no constraints" << std::endl;
    VectorXd deltax;
    while(1){
        deltax = alpha*calcdeltax_sf(x);
        if(deltax.norm()<threshold){break;}
        x += deltax; 
    }
    ans = x;
    return ans;
}
VectorXd SolveSQP::solve_eqconstraints(VectorXd &x){
    VectorXd ans;
    std::cout << "eq constraints" << std::endl;
    return ans;
}
VectorXd SolveSQP::solve_ineqconstraints(VectorXd &x){
    VectorXd ans;
    return ans;
}
VectorXd SolveSQP::solve_bothconstraints(VectorXd &x){
    VectorXd ans;
    return ans;
}

#if defined(SOLV_IS_MAIN)

class squareFuncvec : public Funcvec{
    public:
        squareFuncvec();
        VectorXd function(VectorXd x);
};

squareFuncvec::squareFuncvec(){
    xvecsize = 2;
}

VectorXd squareFuncvec::function(VectorXd x){
    VectorXd ans = VectorXd::Zero(1);
    ans(0) =  (x(0) + 1)*(x(0) + 1) + (x(1) + 5)*(x(1) + 5); 
    return ans;
}

class linearconstraints : public Funcvec{
    public:
        VectorXd function(VectorXd x);
};

VectorXd linearconstraints::function(VectorXd x){
    VectorXd ans = VectorXd::Zero(1);//x*x + y*y + 2*x*y
    ans(0) = x(1) -x(0);
    return ans;
}

int main(){
    squareFuncvec sq;
    linearconstraints linx;
    VectorXd x(2);
    x<<2,3;
    //SolveSQP ssqp(&sq,&linx,SolveNU::EQCONSTRAINT);
    SolveSQP ssqp(&sq);
    std::cout << "sq " << ssqp.calcoptfunc(x) << std::endl;
    VectorXd hogex = ssqp.solve(x);//ssqp.constrainteqh(x);//ssqp.constrainteqh(x);
    std::cout << " solve: " << std::endl;
    showvec(hogex);
    std::cout << "sq " << ssqp.calcoptfunc(hogex) << std::endl;
}
#endif