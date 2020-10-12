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

class ArioID{
public:
    typedef std::vector<int> int_vector;
    typedef std::vector<double> double_vector;
public:
    ArioID();
    double_vector const& getforce(double_vector const &angle,double_vector const &tau) const {
        return angle;
    }
    VectorXd stdvectoeig(const double_vector &stv);
    double_vector eigtostdvec(VectorXd eig);
private:
    int_vector v_;
    double_vector vd_;
    invdSolvenu *maninvd;
};

ArioID::ArioID(){
    int ii,jointn = 7;
    maninvd = new invdSolvenu(jointn);
    /*set RTCRANE DH*/
    maninvd->setdhparameter(0,0.0d*M_PI,0.0d,0.064d,-0.5d*M_PI);//(int num,double thoff,double aa,double di,double alph);
    maninvd->setdhparameter(1,0.0d*M_PI,0.0d,0.0d,0.5d*M_PI);//(int num,double thoff,double aa,double di,double alph);
    maninvd->setdhparameter(2,0.0d*M_PI,0.0d,0.065d+0.185d,-0.5d*M_PI);//(int num,double thoff,double aa,double di,double alph);
    maninvd->setdhparameter(3,0.0d*M_PI,0.0d,0.0d,0.5d*M_PI);//(int num,double thoff,double aa,double di,double alph);
    maninvd->setdhparameter(4,0.0d*M_PI,0.0d,0.121d+0.129d,-0.5d*M_PI);//(int num,double thoff,double aa,double di,double alph);
    maninvd->setdhparameter(5,0.0d*M_PI,0.0d,0.0d,0.5d*M_PI);//(int num,double thoff,double aa,double di,double alph);
    maninvd->setdhparameter(6,0.0d*M_PI,0.0d,0.019d+0.084d,0.0d);//(int num,double thoff,double aa,double di,double alph)
}

VectorXd ArioID::stdvectoeig(const double_vector &stv){
    VectorXd ans(stv.size());
    for(int ii=0;ii<stv.size();ii++){
        ans(ii) = stv[ii];
    }
    return ans;
}

ArioID::double_vector ArioID::eigtostdvec(VectorXd eig){
    double_vector ans(eig.size());
    for(int ii=0;ii<eig.size();ii++){
        ans[ii] = eig(ii);
    }
    return ans;
}

BOOST_PYTHON_MODULE( IDpy ){
    using namespace boost::python;

    class_<ArioID>("ArioID")
        .def("getforce", &ArioID::getforce, return_value_policy<copy_const_reference>());
    to_python_converter<ArioID::int_vector, vector_to_pylist_converter<ArioID::int_vector> >();
    converter::registry::push_back(
        &pylist_to_vector_converter<ArioID::int_vector>::convertible,
        &pylist_to_vector_converter<ArioID::int_vector>::construct,
        boost::python::type_id<ArioID::int_vector>());
    to_python_converter<ArioID::double_vector, vector_to_pylist_converter<ArioID::double_vector> >();
    converter::registry::push_back(
        &pylist_to_vector_converter<ArioID::double_vector>::convertible,
        &pylist_to_vector_converter<ArioID::double_vector>::construct,
        boost::python::type_id<ArioID::double_vector>());
}