/*animat.h*/
#ifndef ANIMAT_H
#define ANMAT_H
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>

typedef Eigen::Matrix<double,8,1> Vector8d;
typedef Eigen::Matrix<double,7,1> Vector7d;
typedef Eigen::Matrix<double,6,1> Vector6d;
typedef Eigen::Matrix<double,5,1> Vector5d;

typedef Eigen::Matrix<double,5,5> Matrix5d;

using namespace Eigen;
namespace fragg{
	extern bool start;
	extern bool view;
    extern bool end;
}
namespace ani{
    void set_axis(void);
    void display(void);
    void init(void);
    void resize(int w,int h);
    void timer(int value);
    void display(void);
    void end(void);
}
void InitialGlut(int ac,char *av[]);
#endif