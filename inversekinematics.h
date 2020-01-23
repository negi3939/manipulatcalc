#ifndef INVKIN_H
#define INVKIN_H
#include "solvenu.h"

class maniSolvenu : public Solvenu {
  protected:
    int jointnum;
    Matrix4d *aA;
    Matrix4d *aAdis;
    Matrix4d *aAtheta;
    Matrix4d *aAthetaoff;
    Matrix4d *aAaal;
    Matrix4d *aAalp;
    double *aal;
    double *alp;
    double *dis;
    double *thetaoff;
    void init();
  public:
    maniSolvenu();
    maniSolvenu(int num);
    VectorXd funcorg(VectorXd x) override;
    void setdhparameter(int num,double thoff,double aa,double di,double alph);
    Vector4d matrixtoquatanion(Matrix4d mat);
    VectorXd getangle(VectorXd x);
    ~maniSolvenu();
};

#endif