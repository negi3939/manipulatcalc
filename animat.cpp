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

double Animation::wide;
pthread_mutex_t Animation::flutex;
pthread_mutex_t Animation::mutex;
pthread_mutex_t Animation::endutex;
static int Animation::start;
static int Animation::view;
static int Animation::end;

Animation::Animation(){
	winnum = 2;
	windowname = new char*[winnum];
	for(int ii = 0;ii<winnum;ii++){
		windowname[ii] = new char[255];
	}
	windowname[0] = {"animation0"};
	windowname[1] = {"animation1"};
	init();
}
void Animation::init(){
	setdispf();
	keyf = Animation::keyboard;
	resiz = Animation::resize;
	timerf = Animation::timerfunc;
	pthread_mutex_init(&anitex,NULL);
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
	glutInitWindowPosition(100,20);
	glutInitWindowSize(1000,1000);
	glutInit(&ac,av);
	glutReshapeFunc(resiz);
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
	while(1){
		//std::cout << "hogehoge" << std::endl;
	}
}
#endif