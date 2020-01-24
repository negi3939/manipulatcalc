/*animat.h*/
#ifndef ANIMAT_H
#define ANMAT_H
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>

class animat{
	protected:
	public:
		void resize(int w,int h);
		void init(void);
		void display();
        void displayrun();
		void end();
		void timer(int value);
		void initialGlut(int ac,char *av[]);
};
void initialGlut(int ac,char *av[]);
#endif