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
#include "animat.h"

#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl


void Animat::resize(int w,int h){
		double wide = 1000.0;
		/*pthread_mutex_lock(&futex);
		wide = ctr::wide;
		pthread_mutex_unlock(&futex);
		*/
		glViewport(0, 0, w, h);
		glLoadIdentity();
		glOrtho(-w / wide, w / wide, -h / wide, h / wide, -1.0, 1.0);
}

void Animat::init(void){
	glClear(GL_COLOR_BUFFER_BIT);
	glClearColor(0.0, 0.0, 0.0, 0.5);
}

void Animat::display(){}

void Animat::displayrun(){
	display();
	glFlush();
}

void Animat::end(){
		bool endf=0;
		/*
		pthread_mutex_lock(&endutex);
		endf = fragg::end;
		pthread_mutex_unlock(&endutex);
		*/
		if(endf==1){exit(0);}
}

void Animat::timer(int value){
		glutPostRedisplay();
		//glutTimerFunc(1,timer,0);
		end();
}



void Animat::initialGlut(int ac,char *av[]){	
	glutInit(&ac,av);
	glutInitWindowPosition(100,20);
	glutInitWindowSize(1000,1000);	
	glutReshapeFunc(staticresize);
	/*
	glutInitDisplayMode(GLUT_RGBA);
	glutCreateWindow("animation");
	glutDisplayFunc(displayrun);
	glutReshapeFunc(resize);
	//glutKeyboardFunc(ctr::keyboard);
	init();
	glutTimerFunc(1,timer,0);
	glutMainLoop();
	*/
}

void resize(int w,int h,int hoge){
		double wide = 1000.0;
		/*pthread_mutex_lock(&futex);
		wide = ctr::wide;
		pthread_mutex_unlock(&futex);
		*/
		glViewport(0, 0, w, h);
		glLoadIdentity();
		glOrtho(-w / wide, w / wide, -h / wide, h / wide, -1.0, 1.0);
		std::cout << "hoge is" << hoge << std::endl;
}

void init(void){
		glClear(GL_COLOR_BUFFER_BIT);
		glClearColor(0.0, 0.0, 0.0, 0.5);
}

void display(){}

void displayrun(){
	display();
	glFlush();
}

void end(){
		bool endf=0;
		/*
		pthread_mutex_lock(&endutex);
		endf = fragg::end;
		pthread_mutex_unlock(&endutex);
		*/
		if(endf==1){exit(0);}
}

void timer(int value){
		glutPostRedisplay();
		glutTimerFunc(1,timer,0);
		end();
}



void initialGlut(int ac,char *av[]){	
	glutInit(&ac,av);
	glutInitWindowPosition(100,20);
	glutInitWindowSize(1000,1000);
	glutReshapeFunc(resize);
	glutInitDisplayMode(GLUT_RGBA);
	glutCreateWindow("animation");
	glutDisplayFunc(displayrun);
	glutReshapeFunc(resize);
	//glutKeyboardFunc(ctr::keyboard);
	init();
	glutTimerFunc(1,timer,0);
	glutMainLoop();
}