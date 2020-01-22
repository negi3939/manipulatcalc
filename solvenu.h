#ifndef SOLVENU_H
#define SOLVENU_H

using namespace Mymath;

class Solvenu : public Funcvec{
    protected:
        VectorXd targetfx;
        VectorXd x;
    public:
        void settargetfx(VectorXd tfx);
        VectorXd gettargetfx();
        VectorXd function(VectorXd x) override;
        virtual VectorXd funcorg(VectorXd x);
        VectorXd functionerror(VectorXd x);
        VectorXd solve(VectorXd intx);
};

#endif