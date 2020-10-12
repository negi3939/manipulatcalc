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
    double_vector const& get_list(double_vector const &a) const {
        return a;
    }
private:
    int_vector v_;
    double_vector vd_;
    invdSolvenu *maninvd;
};

ArioID::ArioID(){
    int ii,jointn = 7;
    maninvd = new invdSolvenu(jointn);
}


BOOST_PYTHON_MODULE( IDpy ){
    using namespace boost::python;

    class_<ArioID>("ArioID")
        .def("get_list", &ArioID::get_list, return_value_policy<copy_const_reference>());
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