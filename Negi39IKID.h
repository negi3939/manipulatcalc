#ifndef NEGI39IKID_H
#define NEGI39IKID_H

#include "mymath.h"
#include "solvenu.h"
#include "inversekinematics.h"
#include "inversedynamics.h"

#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl
using namespace Mymath;

enum EULERFL{
    NON=0,
    ROLL=1,
    PITCH=2,
    YAW=3,
};

enum ANGLEUNIT{
    RAD=0,
    DEG=1,
};

class DHparameter{
    private:
        double thetaoffset;
        double a;
        double d;    
        double alpha;
    public:
        void set(double l_thetaoffset,double l_a,double l_d,double l_alpha);
        double getthetaoffset();
        double geta();
        double getd();
        double getalpha();
};

class Negi39IKID{
    protected:
        int jointnum;//関節数
        LIMITFlag limitfl;//limitがあるか
        std::vector<DHparameter> dh;//DHparamter
        invkSolvenu *maninvk;//IKsolver
        invdSolvenu *maninvd;//IDsolver
        Matrix4d mattheta;//回転並進行列
        VectorXd angle_rad;//rad(1~6軸)
        VectorXd angle_deg;//deg(1~6軸)
        VectorXd angle_defo;//deg(1~6軸) デフォルト姿勢(rad)
        VectorXd targetx;//目標位置姿勢
        Vector4d qua;//クオータニオン
        Vector3d pos;//3軸位置
        VectorXd uplimit;//関節可動範囲上限
        VectorXd lowlimit;//関節可動範囲下限
    public:
        virtual Negi39IKID();
        Negi39IKID(std::vector<DHparameter> l_dh);
        Negi39IKID(std::vector<DHparameter> l_dh,VectorXd l_uplimit,VectorXd l_lowlimit);
        VectorXd radtodeg(VectorXd rad);
        VectorXd degtorad(VectorXd deg);
        VectorXd solve_relative(Vector3d target,EULERFL eulfl,double eulangle,ANGLEUNIT unit=RAD);
        VectorXd solve_abstarg(VectorXd target,ANGLEUNIT unit=RAD);
        VectorXd setangle(VectorXd ang);
        void init();
        void setdefoko(VectorXd defo,ANGLEUNIT unit);
        void setdefoko();
        void show_angle(ANGLEUNIT unit=DEG);
};

#endif