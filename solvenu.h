#ifndef SOLVENU_H
#define SOLVENU_H

#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl
using namespace Mymath;

enum LIMITFlag{
    LIMITOFF=0,
    LIMITON=1,
};

class Solvenu : public Funcvec{
    protected:
        long countlimit;
        VectorXd targetfx;
        VectorXd x;
        VectorXd upperlimit;
        VectorXd lowerlimit;
        LIMITFlag limitfl;
    public:
        Solvenu();
        void setcountlimit(long a);
        void setlimit(VectorXd uplimit,VectorXd lowlimit);
        int checklimit(VectorXd &x);
        VectorXd sigmoidlimit(VectorXd &x,double alpha);
        VectorXd penaltyfunc(VectorXd &x);
        void settargetfx(VectorXd tfx);
        VectorXd gettargetfx();
        VectorXd function(VectorXd x) override;
        virtual VectorXd funcorg(VectorXd x);
        VectorXd functionerror(VectorXd x);
        VectorXd solve(VectorXd intx);
        ~Solvenu();
};

#endif