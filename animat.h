#ifndef ANIMATION_H
#define ANIMATION_H

#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl
typedef Eigen::Matrix<double,8,1> Vector8d;
typedef Eigen::Matrix<double,7,1> Vector7d;
typedef Eigen::Matrix<double,6,1> Vector6d;
typedef Eigen::Matrix<double,5,1> Vector5d;
typedef Eigen::Matrix<double,5,5> Matrix5d;

using namespace Eigen;
class Structthis{
	public:
		void *num;
		void *send;
		void *instthis;
};

class Animation{
	protected:
		int winnum;
		char **windowname;
		pthread_t anithread;
		pthread_mutex_t anitex;
		void (*keyf)(unsigned char,int,int);
		void (*resiz)(int,int);
		void (*disp)();
		void (*timerf)(int);
		void aniloop(void* send);
		void g_init(void);
		virtual void setdispf();
	public:
		Animation();
		void init();
	private:
		static double wide;
		static pthread_mutex_t flutex;
		static pthread_mutex_t mutex;
		static pthread_mutex_t endutex;
		static int start;
		static int view;
		static int end;
		static void* launchthread(void *pParam){
        	reinterpret_cast<Animation*>(reinterpret_cast<Structthis*>(pParam)->instthis)->aniloop(reinterpret_cast<Structthis*>(pParam)->send);
        	pthread_exit(NULL);
    	}
		static void keyboard(unsigned char key, int x, int y){
			double u;
			switch(key){
			case '-':
				pthread_mutex_lock(&flutex);
				wide -= 20;
				std::cout<<"////////zoom out("<<wide<<")\\\\\\"<<std::endl;
				pthread_mutex_unlock(&flutex);
				break;
			case '+':
				pthread_mutex_lock(&flutex);
				wide += 20;
				std::cout<<"\\\\\\\\zoom in("<<wide<<"///////"<<std::endl;
				pthread_mutex_unlock(&flutex);
				break;
			case 's':
				std::cout<<"========start=========="<<std::endl;
				pthread_mutex_lock(&flutex);
				start= 1;
				pthread_mutex_unlock(&flutex);
				break;
			case 'v':
				if(view==0){
					pthread_mutex_lock(&flutex);
					view= 1;
					pthread_mutex_unlock(&flutex);
					std::cout<<"=======view========="<<std::endl;
					break;
				}else{
					pthread_mutex_lock(&flutex);
					view= 0;
					pthread_mutex_unlock(&flutex);
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
	static void resize(int w,int h){
		double l_wide;
		pthread_mutex_lock(&flutex);
		l_wide = wide;
		pthread_mutex_unlock(&flutex);
		glViewport(0, 0, w, h);
		glLoadIdentity();
		glOrtho(-w / wide, w / wide, -h / wide, h / wide, -1.0, 1.0);
	}
	static void display(){
			glClear(GL_COLOR_BUFFER_BIT);
			usleep(10000);
			glFlush();
	}
	static void timerfunc(int value){
		glutPostRedisplay();
		glutTimerFunc(1,timerfunc,0);
		endfunc();
	}
	static void endfunc(){
		bool endf;
		pthread_mutex_lock(&endutex);
		endf = end;
		pthread_mutex_unlock(&endutex);
		if(endf==1){exit(0);}
	}
};

#endif