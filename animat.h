/*animat.h*/
#ifndef ANIMAT_H
#define ANMAT_H
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>

class Animat{
	protected:
		static void* statictimer(void *pParam,int val) {
        	reinterpret_cast<Animat*>(pParam)->timer(val);
      	}
		static void* staticresize(void *pParam,int w,int h) {
        	reinterpret_cast<Animat*>(pParam)->resize(w,h);
      	}
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