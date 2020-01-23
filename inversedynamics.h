#ifndef INVDYN_H
#define INVDYN_H
#include "solvenu.h"
#include "inversekinematics.h"

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

#endif