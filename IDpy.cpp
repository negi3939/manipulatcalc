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
#include <vector>
#include <boost/python.hpp>
#include <boost/foreach.hpp>
#include <boost/range/value_type.hpp>

#include "mymath.h"
#include "solvenu.h"
#include "inversekinematics.h"
#include "inversedynamics.h"
#include "IDpy.h"

Negi39FIKFID::Negi39FIKFID(){
    int ii,jointn = 7;
    maninvk = new invkSolvenu(jointn);
    maninvd = new invdSolvenu(jointn);
    /*set RTCRANE DH*/
    maninvk->setdhparameter(0,0.0d*M_PI,0.0d,0.064d,-0.5d*M_PI);//(int num,double thoff,double aa,double di,double alph);
    maninvk->setdhparameter(1,0.0d*M_PI,0.0d,0.0d,0.5d*M_PI);//(int num,double thoff,double aa,double di,double alph);
    maninvk->setdhparameter(2,0.0d*M_PI,0.0d,0.065d+0.185d,-0.5d*M_PI);//(int num,double thoff,double aa,double di,double alph);
    maninvk->setdhparameter(3,0.0d*M_PI,0.0d,0.0d,0.5d*M_PI);//(int num,double thoff,double aa,double di,double alph);
    maninvk->setdhparameter(4,0.0d*M_PI,0.0d,0.121d+0.129d,-0.5d*M_PI);//(int num,double thoff,double aa,double di,double alph);
    maninvk->setdhparameter(5,0.0d*M_PI,0.0d,0.0d,0.5d*M_PI);//(int num,double thoff,double aa,double di,double alph);
    maninvk->setdhparameter(6,0.0d*M_PI,0.0d,0.019d+0.084d,0.0d);//(int num,double thoff,double aa,double di,double alph)
    maninvd->copy(maninvk);
    forcemom.resize(6);
    jointtau.resize(jointn);
    jointangle = VectorXd::Zero(jointn);
}

Negi39FIKFID::~Negi39FIKFID(){
    delete maninvk;
    delete maninvd;
}

void Negi39FIKFID::getangle_content(const double_vector &posquo){
    VectorXd posquovector = stdvectoeig(posquo);
    Vector4d quatanion;
    quatanion = posquovector.block(3,0,4,1);
    if(quatanion.norm()<0.8d||quatanion.norm()>1.2d){
        std::cout << "quatanion is wrong. set appropriate value." << std::endl;
        exit(0);
    }
    maninvk->settargetfx(posquovector);
    calcik();
    joint_angle = eigtostdvec(jointangle);
}
void Negi39FIKFID::getpos_content(const double_vector &angle){
    stdvectoeig(angle,jointangle);
    Vector4d quatanion;
    Vector3d position;
    Matrix4d mattheta;
    VectorXd poq = VectorXd::Zero(7);
    poq(3) = 1.0d;
    maninvk->calcaA(jointangle,mattheta);
    quatanion = maninvk->matrixtoquatanion(mattheta);
    position = mattheta.block(0,3,3,1);
    poq.block(0,0,3,1) = position;
    poq.block(3,0,4,1) = quatanion;
    pos_qua = eigtostdvec(poq);
}
void Negi39FIKFID::getforce_content(const double_vector &angle,const double_vector &tau){
    stdvectoeig(angle,jointangle);
    VectorXd jointtau = stdvectoeig(tau);
    Vector3d forcev,momentv;
    maninvd->calcaA(jointangle);
    maninvd->calcforce(jointtau,forcev,momentv);
    VectorXd fm = VectorXd::Zero(6);
    fm.block(0,0,3,1) = forcev;
    fm.block(3,0,3,1) = momentv;
    forcemom = eigtostdvec(fm); 
}

void Negi39FIKFID::gettau_content(const double_vector &angle,const double_vector &fm){
    stdvectoeig(angle,jointangle);
    VectorXd formom = stdvectoeig(fm);
    Vector3d forcev,momentv;
    forcev = formom.block(0,0,3,1);
    momentv = formom.block(3,0,3,1);
    maninvd->calcaA(jointangle);
    VectorXd tau = maninvd->gettau(forcev,momentv);
    jointtau = eigtostdvec(tau);
}

void Negi39FIKFID::setjointnum(const double &jj){
    int ii,jointn = jj;
    delete maninvk;
    delete maninvd;
    maninvk = new invkSolvenu(jointn);
    maninvd = new invdSolvenu(jointn);
    std::cout << "set JOINT_NUM :" << maninvd->getjointnum() << std::endl;
    jointtau.resize(jointn);
}

void Negi39FIKFID::setdhparameter(const int &ii,const double_vector &dh){
    maninvk->setdhparameter(ii,dh[0],dh[1],dh[2],dh[3]);
    maninvd->setdhparameter(ii,dh[0],dh[1],dh[2],dh[3]);
    std::cout << "set Dh : " << ii << "th joint -> thetaoffset: " << maninvd->getthetaoff(ii) << " [rad]  a: " << maninvd->getaal(ii) << " [m] d: " << maninvd->getdis(ii) << " [m] alpha:"<< maninvd->getalp(ii) << " [rad]"<<std::endl;
}

void Negi39FIKFID::calcik(){
    jointangle = maninvk->getangle(jointangle);
}

void Negi39FIKFID::stdvectoeig(const double_vector &stv,VectorXd &eig){
    if(eig.size()!=stv.size()){
        std::cout << "size is not match" << std::endl;
        exit(0);
    }
    for(int ii=0;ii<stv.size();ii++){
        eig(ii) = stv[ii];
    }
}

VectorXd Negi39FIKFID::stdvectoeig(const double_vector &stv){
    VectorXd ans(stv.size());
    for(int ii=0;ii<stv.size();ii++){
        ans(ii) = stv[ii];
    }
    return ans;
}

Negi39FIKFID::double_vector Negi39FIKFID::eigtostdvec(VectorXd eig){
    double_vector ans;
    ans.resize(eig.size());
    for(int ii=0;ii<eig.size();ii++){
        ans[ii] = eig(ii);
    }
    return ans;
}

BOOST_PYTHON_MODULE( IDpy ){
    using namespace boost::python;

    class_<Negi39FIKFID>("Negi39FIKFID")
        .def("setjointnum", &Negi39FIKFID::setjointnum)
        .def("setdhparameter", &Negi39FIKFID::setdhparameter)
        .def("getangle", &Negi39FIKFID::getangle, return_value_policy<copy_const_reference>())
        .def("getpos", &Negi39FIKFID::getpos, return_value_policy<copy_const_reference>())
        .def("getforce", &Negi39FIKFID::getforce, return_value_policy<copy_const_reference>())
        .def("gettau", &Negi39FIKFID::gettau, return_value_policy<copy_const_reference>());
    to_python_converter<Negi39FIKFID::int_vector, vector_to_pylist_converter<Negi39FIKFID::int_vector> >();
    converter::registry::push_back(
        &pylist_to_vector_converter<Negi39FIKFID::int_vector>::convertible,
        &pylist_to_vector_converter<Negi39FIKFID::int_vector>::construct,
        boost::python::type_id<Negi39FIKFID::int_vector>());
    to_python_converter<Negi39FIKFID::double_vector, vector_to_pylist_converter<Negi39FIKFID::double_vector> >();
    converter::registry::push_back(
        &pylist_to_vector_converter<Negi39FIKFID::double_vector>::convertible,
        &pylist_to_vector_converter<Negi39FIKFID::double_vector>::construct,
        boost::python::type_id<Negi39FIKFID::double_vector>());
}