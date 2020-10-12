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

class stddvector {
public:
    typedef std::vector<int> int_vector;
    typedef std::vector<double> double_vector;
public:
    double_vector const& get_list(double_vector const &a) const {
        return a;
    }
private:
    int_vector v_;
    double_vector vd_;
};


BOOST_PYTHON_MODULE( IDpy ){
    using namespace boost::python;

    class_<stddvector>("stddvector")
        .def("get_list", &stddvector::get_list, return_value_policy<copy_const_reference>());
    to_python_converter<stddvector::int_vector, vector_to_pylist_converter<stddvector::int_vector> >();
    converter::registry::push_back(
        &pylist_to_vector_converter<stddvector::int_vector>::convertible,
        &pylist_to_vector_converter<stddvector::int_vector>::construct,
        boost::python::type_id<stddvector::int_vector>());
    to_python_converter<stddvector::double_vector, vector_to_pylist_converter<stddvector::double_vector> >();
    converter::registry::push_back(
        &pylist_to_vector_converter<stddvector::double_vector>::convertible,
        &pylist_to_vector_converter<stddvector::double_vector>::construct,
        boost::python::type_id<stddvector::double_vector>());
}