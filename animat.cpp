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
#include"figure.h"

#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl


pthread_mutex_t mutex;
pthread_mutex_t futex;
pthread_mutex_t endutex;

namespace fragg{
	bool start;
	bool view;
	bool end;
}

namespace ctr{
	double wide = 1000.0;
	void keyboard(unsigned char key, int x, int y){
		double u;
		switch(key){
		case '-':
			pthread_mutex_lock(&futex);
			wide -= 20;
			std::cout<<"////////zoom out("<<wide<<")\\\\\\"<<std::endl;
			pthread_mutex_unlock(&futex);
			break;
		case '+':
			pthread_mutex_lock(&futex);
			wide += 20;
			std::cout<<"\\\\\\\\zoom in("<<wide<<"///////"<<std::endl;
			pthread_mutex_unlock(&futex);
			break;
		case 's':
			std::cout<<"========start=========="<<std::endl;
			pthread_mutex_lock(&futex);
			fragg::start= 1;
			pthread_mutex_unlock(&futex);
			break;
		case 'v':
			if(fragg::view==0){
			pthread_mutex_lock(&futex);
			fragg::view= 1;
			pthread_mutex_unlock(&futex);
			std::cout<<"=======view========="<<std::endl;
			break;
			}else{
				pthread_mutex_lock(&futex);
				fragg::view= 0;
				pthread_mutex_unlock(&futex);
				std::cout<<"=======no view========="<<std::endl;
				break;
			}
		case 'q':
			exit(0);
			break;
		default:
			break;
		}
	}

}

namespace ani{

	static void DrawString(char str[], double w, double h, double x0, double y0){
		glDisable(GL_LIGHTING);
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		gluOrtho2D(0, w, h, 0);
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();
		glRasterPos2d(x0, y0);
		int i;
		for(i = 0; str[i]!='\0'; ++i){
			char ic = str[i];
			glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, ic);
			//glutBitmapCharacter(GLUT_BITMAP_9_BY_15, ic);
		}
		
		glPopMatrix(); 
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);
	}

	void resize(int w,int h){
		double wide;
		pthread_mutex_lock(&futex);
		wide = ctr::wide;
		pthread_mutex_unlock(&futex);
		glViewport(0, 0, w, h);
		glLoadIdentity();
		glOrtho(-w / wide, w / wide, -h / wide, h / wide, -1.0, 1.0);
	}
	
	void set_axis(void){	
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
		
	}

	
	void display(){
		glClear(GL_COLOR_BUFFER_BIT);
		set_axis();
		usleep(10000);
		glFlush();
		resize(1000.0,1000.0);

	}
	
	void init(void){
		glClearColor(0.0, 0.0, 0.0, 0.5);
	}

	void timer(int value){
		glutPostRedisplay();
		glutTimerFunc(1,timer,0);
		end();
	}

	void end(){
		bool endf;
		pthread_mutex_lock(&endutex);
		endf = fragg::end;
		pthread_mutex_unlock(&endutex);
		if(endf==1){exit(0);}
	}
}

void InitialGlut(int ac,char *av[]){
	glutInit(&ac,av);
	glutInitWindowPosition(100,20);
	glutInitWindowSize(1000,1000);
	glutReshapeFunc(ani::resize);
	glutInitDisplayMode(GLUT_RGBA);
	glutCreateWindow("animation");
	glutDisplayFunc(ani::display);
	glutReshapeFunc(ani::resize);
	glutKeyboardFunc(ctr::keyboard);
	ani::init();
	glutTimerFunc(1,ani::timer,0);
	glutMainLoop();
}

class Animation{
	private:
		static void* launchThread(void *pParam) {
        	reinterpret_cast<Animation*>(pParam)->aniloop();
        	pthread_exit(NULL);
    	}
	protected:
		int winnum;
		char **windowname;
		double wide;
		pthread_t anithread;
		pthread_mutex_t anitex;
		void aniloop();
	public:
		Animation();
		void init();
};

Animation::Animation(){
	winnum = 2;
	windowname = new char*[winnum];
	for(int ii = 0;ii<winnum;ii++){
		windowname[ii] = new char[255];
	}
	windowname[0] = {"hoge animation"};
	init();
}
void Animation::init(){
	pthread_mutex_init(&anitex,NULL);
	pthread_create(&anithread,NULL,Animation::launchThread,this);
}

void Animation::aniloop(){
	int ac = 1;
	char *av[5];
	glutInit(&ac,av);
	glutInitWindowPosition(100,20);
	glutInitWindowSize(1000,1000);
	glutReshapeFunc(ani::resize);
	glutInitDisplayMode(GLUT_RGBA);
	glutCreateWindow(windowname[0]);
	glutDisplayFunc(ani::display);
	glutReshapeFunc(ani::resize);
	glutKeyboardFunc(ctr::keyboard);
	ani::init();
	glutTimerFunc(1,ani::timer,0);
	glutMainLoop();
}


#if defined(ANIME_IS_MAIN)
int main(){
	Animation ani;
	while(1){}
}
#endif