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

template<typename T_>
class vector_to_pylist_converter {
public:
    typedef T_ native_type;
    static PyObject* convert(native_type const& v) {
        namespace py = boost::python;
        py::list retval;
        BOOST_FOREACH(typename boost::range_value<native_type>::type i, v)
        {
            retval.append(py::object(i));
        }
        return py::incref(retval.ptr());
    }
};

template<typename T_>
class pylist_to_vector_converter {
public:
    typedef T_ native_type;
    static void* convertible(PyObject* pyo) {
        if (!PySequence_Check(pyo))
            return 0;

        return pyo;
    }
    static void construct(PyObject* pyo, boost::python::converter::rvalue_from_python_stage1_data* data){
        namespace py = boost::python;
        native_type* storage = new(reinterpret_cast<py::converter::rvalue_from_python_storage<native_type>*>(data)->storage.bytes) native_type();
        for (py::ssize_t i = 0, l = PySequence_Size(pyo); i < l; ++i) {
            storage->push_back(
                py::extract<typename boost::range_value<native_type>::type>(
                    PySequence_GetItem(pyo, i)));
        }
        data->convertible = storage;
    }
};

class ArioID{
    public:
        typedef std::vector<int> int_vector;
        typedef std::vector<double> double_vector;
    protected:
        invkSolvenu *maninvd;
        double_vector forcemom;
        double_vector jointangle;
        double_vector jointtau;
        VectorXd angle;
        VectorXd ctauv;
        Matrix4d mattheta;
        Vector3d forcev,momentv;
    public:
        ArioID();
        double_vector const& getforcemoment(double_vector const& ang,double_vector const& tau) const {
            return forcemom;
         }
        double_vector const& gettau(double_vector const& ang,double_vector const& fm) const {
            return jointtau;
        }
        
};

ArioID::ArioID(){
    int ii,jointn = 7;
    invdSolvenu *maninvd;
    maninvd = new invdSolvenu(jointn);
    /*RT CRANE*/
    maninvd->setdhparameter(0,0.0d*M_PI,0.0d,0.064d,-0.5d*M_PI);//(int num,double thoff,double aa,double di,double alph);
    maninvd->setdhparameter(1,0.0d*M_PI,0.0d,0.0d,0.5d*M_PI);//(int num,double thoff,double aa,double di,double alph);
    maninvd->setdhparameter(2,0.0d*M_PI,0.0d,0.065d+0.185d,-0.5d*M_PI);//(int num,double thoff,double aa,double di,double alph);
    maninvd->setdhparameter(3,0.0d*M_PI,0.0d,0.0d,0.5d*M_PI);//(int num,double thoff,double aa,double di,double alph);
    maninvd->setdhparameter(4,0.0d*M_PI,0.0d,0.121d+0.129d,-0.5d*M_PI);//(int num,double thoff,double aa,double di,double alph);
    maninvd->setdhparameter(5,0.0d*M_PI,0.0d,0.0d,0.5d*M_PI);//(int num,double thoff,double aa,double di,double alph);
    maninvd->setdhparameter(6,0.0d*M_PI,0.0d,0.019d+0.084d,0.0d);//(int num,double thoff,double aa,double di,double alph);
}

BOOST_PYTHON_MODULE( ArioID ){
    using namespace boost::python;
    class_<ArioID>("ArioID")
        .def("getforcemoment", (void(ArioID::*)(ArioID::double_vector const&,ArioID::double_vector const&))&ArioID::getforcemoment, return_value_policy<copy_const_reference>())
        .def("gettau", (void(ArioID::*)(ArioID::double_vector const&,ArioID::double_vector const&))&ArioID::gettau, return_value_policy<copy_const_reference>());
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