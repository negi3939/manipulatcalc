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
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include"animat.h"
#include"figure.h"

class Robotviz : public Animation{
    protected:
        void setdispf();//displayの関数を設定
    public:
        Robotviz();
    protected:
    static void display(){
			glClear(GL_COLOR_BUFFER_BIT);
            Vector3d c(1.0d,1.0d,1.0d);
		    Vector2d stpx(-20.0d,0.0d),edpx(20.0d,0.0d);
		    Vector2d stpy(0.0d,-20.0d),edpy(0.0d,20.0d);
		    Vector2d rp;
		    draw::line xaxis,yaxis,r;
		    draw::curve theta;
		    int n = 1,m = 1;
		    Vector2d zero;
		    zero = MatrixXd::Zero(2, 1);
		    xaxis.setcolor(c);
		    yaxis.setcolor(c);
		    xaxis.setsp(stpx);
		    yaxis.setsp(stpy);
		    xaxis.setep(edpx);
		    yaxis.setep(edpy);
		    xaxis.dr();
		    yaxis.dr();

		for(n=1;n<6;n++){
			theta.setcolor(c);
			theta.setr((double)n/5);
			theta.setp(zero);
			theta.dr();
		}

		for(m=1;m<12;m++){
			r.setcolor(c);
			rp(0) = 1.0*cos((double)m*M_PI/6.0);
			rp(1) = 1.0*sin((double)m*M_PI/6.0);
			r.setsp(zero);
			r.setep(rp);
			r.dr();
		}
            glEnd();
            glFlush();
	}
};
Robotviz::Robotviz(){
    setdispf();
}

void Robotviz::setdispf(){
    disp = Robotviz::display;
}

#if defined(VIZ_IS_MAIN)
int main(){
    Robotviz rv;
    rv.start();
    while(1){}
}
#endif