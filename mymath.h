#ifndef MYMATH_H
#define MYMATH_H
using namespace Eigen;

namespace Mymath{
	class Funcvec{
		public:
			virtual VectorXd function(VectorXd x);	
	};
	double sign(double A);
	/*
	void eig(MatrixXd aA,MatrixXd bB,MatrixXd &eigenV,MatrixXd &eigenD);
	void eig(MatrixXd aA,MatrixXd &eigenV,MatrixXd &eigenD);
	*/
	MatrixXd inv(MatrixXd aA);
	MatrixXd absmat(MatrixXd aA);
	MatrixXd diffvec(VectorXd x,Funcvec *func);
}

#endif