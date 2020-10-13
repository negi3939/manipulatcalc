#ifndef _ID_PY_H_
#define _ID_PY_H_

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

class Negi39FIKFID{
public:
    typedef std::vector<int> int_vector;
    typedef std::vector<double> double_vector;
public:
    Negi39FIKFID();
    ~Negi39FIKFID();
    double_vector const& getangle(double_vector const &posquo) const {//IK
        getangle_content(posquo);
        return joint_angle; 
    }
    double_vector const& getpos(double_vector const &angle) const {
        getpos_content(angle);
        return pos_qua;
    }
    double_vector const& getforce(double_vector const &angle,double_vector const &tau) const {//ID
        getforce_content(angle,tau);
        return forcemom;
    }
    double_vector const& gettau(double_vector const &angle,double_vector const &fm) const {//FD
        gettau_content(angle,fm);
        return jointtau;
    }
    void getangle_content(const double_vector &posquo);
    void getpos_content(const double_vector &angle);
    void getforce_content(const double_vector &angle,const double_vector &tau);
    void gettau_content(const double_vector &angle,const double_vector &fm);
    void setjointnum(const double &jj);
    void setdhparameter(const int &ii,const double_vector &dh);
    void calcik();
    void stdvectoeig(const double_vector &stv,VectorXd &eig);
    VectorXd stdvectoeig(const double_vector &stv);
    double_vector eigtostdvec(VectorXd eig);
    void compositvector(VectorXd a,VectorXd b,VectorXd &ab);
protected:
    int_vector v_;
    double_vector joint_angle;
    double_vector pos_qua;
    double_vector forcemom;
    double_vector jointtau;
    VectorXd jointangle;
    invkSolvenu *maninvk;
    invdSolvenu *maninvd;
};

#endif