#include<iostream>
#include<string>
#include<iomanip>
#include<fstream>
#include<sstream>
#include<cstdlib>
#include<math.h>
#include<unistd.h>
#include<pthread.h>
#include<Eigen/Dense>
#include<Eigen/Core>
#include<Eigen/Geometry>
#include<Eigen/StdVector>
#include <Eigen/LU>
#include <functional> //-std=c++11をつける

#define DT                       (0.0000001)                        //時間ステップ(s)
#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl

using namespace Eigen;

namespace Mymath{
	double sign(double A){
    	if(A>0) return 1;
    	else if(A<0) return -1;
    	else return 0;
	}

	void eig(MatrixXd aA,MatrixXd bB,MatrixXd &eigenV,MatrixXd &eigenD){
		if((aA.rows()!=aA.cols())||(bB.rows()!=bB.cols())||(aA.rows()!=bB.rows())){std::cout << "Matrix size is not same."<<std::endl;}
		int ii,jj,size = aA.rows();
		GeneralizedEigenSolver<MatrixXd> solve(aA,bB);
   		MatrixXcd Dc = solve.eigenvalues();
   		VectorXd Dv = Dc.real();
   		eigenD = Dv.asDiagonal();
   		MatrixXcd Vc = solve.eigenvectors();
   		MatrixXd Vbuf = Vc.real();
		eigenV = MatrixXd::Zero(size,size);
		for(jj=0;jj<size;jj++){
			eigenV(size-1,jj) = pow(-1.0,(double)(jj+1));
			for(ii=0;ii<size-1;ii++){
				eigenV(ii,jj) = Vbuf(ii,jj)/(Vbuf(size-1,jj)/eigenV(size-1,jj));
			}
		}
		for(ii=0;ii<size;ii++){
			VectorXd buff = aA*eigenV.block(0,ii,size,1) - eigenD(ii,ii)*bB*eigenV.block(0,ii,size,1);
			for(int jj=0;jj<size;jj++){
				if(abs(buff(jj))>0.1){
				std::cout << "eigen val or eigen vector is not correct." << std::endl; 
			}
			}
		}	
	}
	void eig(MatrixXd aA,MatrixXd &eigenV,MatrixXd &eigenD){
		if(aA.rows()!=aA.cols()){std::cout << "Matrix size is not same."<<std::endl;}
		int size = aA.rows();
		EigenSolver<MatrixXd> solve(aA);
   		MatrixXcd Dc = solve.eigenvalues();
   		VectorXd Dv = Dc.real();
   		eigenD = Dv.asDiagonal();
   		MatrixXcd Vc = solve.eigenvectors();
   		eigenV = Vc.real();
		for(int ii=0;ii<size;ii++){
			VectorXd buff = aA*eigenV.block(0,ii,size,1) - eigenD(ii,ii)*eigenV.block(0,ii,size,1);
			for(int jj=0;jj<size;jj++){
				if(abs(buff(jj))>0.1){
				std::cout << "eigen val or eigen vector is not correct." << std::endl; 
			}
			}
		}	
	}

	MatrixXd inv(MatrixXd aA){
		FullPivLU< MatrixXd > invM(aA);
		return invM.inverse();
	}

	MatrixXd absmat(MatrixXd aA){
		int ii,jj;
		MatrixXd ans = MatrixXd::Zero(aA.rows(),aA.cols()); 
		for(ii=0;ii<aA.rows();ii++){
	   		for(jj=0;jj<aA.cols();jj++){
		  		ans(ii,jj) = std::abs(aA(ii,jj));  
   			}
   		}
		return ans;
	}

}