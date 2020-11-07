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
#include"figure.h"
#include"animat.h"


double *Animation::wide;
pthread_mutex_t Animation::flutex;
pthread_mutex_t Animation::mutex;
pthread_mutex_t Animation::endutex;
static int Animation::startfl;
static int Animation::viewfl;
static int Animation::endfl;


Animation::Animation(){
	winnum = 2;
	windowname = new char*[winnum];
	for(int ii = 0;ii<winnum;ii++){
		windowname[ii] = new char[255];
	}
	wide = new double;
	(*wide) = 1000;
	windowname[0] = {"animation0"};
	windowname[1] = {"animation1"};
}
void Animation::start(){
	setdispf();
	keyf = Animation::keyboard;
	resiz = Animation::resize;
	timerf = Animation::timerfunc;
	pthread_mutex_init(&anitex,NULL);
	pthread_mutex_init(&flutex,NULL);
	pthread_mutex_init(&mutex,NULL);
	pthread_mutex_init(&endutex,NULL);
	Structthis *anisend = new Structthis;
	anisend->instthis = (void*)this;
	pthread_create(&anithread,NULL,Animation::launchthread,anisend);
}

void Animation::setdispf(){
	disp = Animation::display;
}

void Animation::g_init(void){
		glClearColor(0.0, 0.0, 0.0, 0.5);
}

void Animation::aniloop(void *send){
	int ac = 1;
	char *av[5];
	glutInit(&ac,av);
	glutInitWindowPosition(100,20);
	glutInitWindowSize(1000,1000);
	glutInitDisplayMode(GLUT_RGBA);
	glutCreateWindow(windowname[0]);
	glutDisplayFunc(disp);
	glutReshapeFunc(resiz);
	glutKeyboardFunc(keyf);
	g_init();
	glutTimerFunc(1,timerf,0);
	glutMainLoop();
}

#if defined(ANIME_IS_MAIN)
int main(){
	Animation ani;
	ani.start();
	while(1){
		//std::cout << "hogehoge" << std::endl;
	}
}
#endif