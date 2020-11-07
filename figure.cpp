/*figure.cpp*/
/*図形描写用*/
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

namespace draw{
	class line{
	private:
		Vector3d color;
		Vector2d sp;
		Vector2d ep;
	public:
		void setcolor(const Vector3d &iro);
		void setsp(const Vector2d &s);
		void setep(const Vector2d &e);
		void dr(void);
	};
	
	void line::setcolor(const Vector3d &iro){
		color = iro;
	}
	void line::setsp(const Vector2d &s){
		sp = s;
	}
	void line::setep(const Vector2d &e){
		ep = e;
	}
	void line::dr(void){
		glColor3d(color.x(),color.y(),color.z());
		glLineWidth(0.1);
		glBegin(GL_LINE_LOOP);
		glVertex2d(sp.x(),sp.y());
		glVertex2d(ep.x(),ep.y());
		glEnd();
		
	}
	
	class line3d :public line{
		private:
			Vector3d color;
			Vector3d sp;
			Vector3d ep;
			double width;
		public:
			line3d();
			void setcolor(const Vector3d &iro);
			void setsp(const Vector3d &s);
			void setep(const Vector3d &e);
			void setth(const double &w);
			void dr(void);
	};

	line3d::line3d(){
		width = 0.1;
	}

	void line3d::setcolor(const Vector3d &iro){
		color = iro;
	}
	void line3d::setsp(const Vector3d &s){
		sp = s;
	}
	void line3d::setep(const Vector3d &e){
		ep = e;
	}

	void line3d::setth(const double &w){
		width = w;
	}

	void line3d::dr(void){
		glColor3d(color.x(),color.y(),color.z());
		glLineWidth(width);
		glBegin(GL_LINE_LOOP);
		glVertex3d(sp.x(),sp.z(),sp.y());
		glVertex3d(ep.x(),ep.z(),ep.y());
		glEnd();
	}

	class link{
	protected:
		Vector3d color;
		Vector2d sp;
		Vector2d ep;
		double think;
	public:
		void setcolor(const Vector3d &iro);
		void setthink(const double &b);
		void setsp(const Vector2d &s);
		void setep(const Vector2d &e);
		void dr(void);
	};	

	void link::setcolor(const Vector3d &iro){
		color = iro;
	}
	void link::setthink(const double &b){
		think = b;
	}
	void link::setsp(const Vector2d &s){
		sp = s;
	}
	void link::setep(const Vector2d &e){
		ep = e;
	}
	void link::dr(void){
		glColor3d(color.x(),color.y(),color.z());
		glBegin(GL_POLYGON);
		double thett;
		thett = std::atan2(sp.y()-ep.y(),sp.x()-ep.x());
		glVertex2d(sp.x()-think/2*sin(thett),sp.y()+think/2*cos(thett));
		glVertex2d(sp.x()+think/2*sin(thett),sp.y()-think/2*cos(thett));
		glVertex2d(ep.x()+think/2*sin(thett),ep.y()-think/2*cos(thett));
		glVertex2d(ep.x()-think/2*sin(thett),ep.y()+think/2*cos(thett));
		glEnd();
		
	}

	class link3d :public link{
		protected:
		Vector3d sp;
		Vector3d ep;
	public:
		void setsp(const Vector3d &s);
		void setep(const Vector3d &e);
		void dr(void);
	};

	void link3d::setsp(const Vector3d &s){
		sp = s;
	}
	void link3d::setep(const Vector3d &e){
		ep = e;
	}
	void link3d::dr(void){
		glColor3d(color.x(),color.y(),color.z());
		glBegin(GL_POLYGON);
		double thett;
		thett = std::atan2(sp.y()-ep.y(),sp.x()-ep.x());
		glVertex3d(sp.x()-think/2*sin(thett),sp.z()-think/2*sin(thett),sp.y()+think/2*cos(thett));
		glVertex3d(sp.x()+think/2*sin(thett),sp.z()+think/2*sin(thett),sp.y()-think/2*cos(thett));
		glVertex3d(ep.x()+think/2*sin(thett),ep.z()+think/2*sin(thett),ep.y()-think/2*cos(thett));
		glVertex3d(ep.x()-think/2*sin(thett),ep.z()-think/2*sin(thett),ep.y()+think/2*cos(thett));
		glEnd();
	}


#define D_C   (160) //bunkarusuu
#define D_CC   (80) //bunkarusuu 
	
	class circle{
		protected:
			Vector3d color;
			double r;
			Vector2d p;
		public:
			void setcolor(const Vector3d &iro);
			void setr(const double &rr);
			void setp(const Vector2d &po);
			void dr(void);
	};

	void circle::setcolor(const Vector3d &iro){
		color = iro;
	}

	void circle::setr(const double &rr){
		r = rr;
	}

	void circle::setp(const Vector2d &po){
		p = po;
	}

	void circle::dr(void){
		glColor3d(color.x(),color.y(),color.z());
		glBegin(GL_POLYGON);
		int i = 0;
		double th;
		for(i=0;i<D_C;i++){
			th = 2.0*M_PI/D_C*(double)i;
			glVertex2d(p.x()+r*cos(th),p.y()+r*sin(th));
		}
		glEnd();
		
	}

	class circle3d : public circle{
		protected:
			Vector3d p;
		public:
			void setp(const Vector3d &po);
			void dr(void);
	};

	void circle3d::setp(const Vector3d &po){
		p = po;
	}

	void circle3d::dr(void){
		glColor3d(color.x(),color.y(),color.z());
		glBegin(GL_POLYGON);
		int i = 0,j=0;
		double th,fa;
		for(j=0;j<D_CC;j++){
			for(i=0;i<D_CC;i++){
				th = 2.0*M_PI/D_CC*(double)i;
				fa = 2.0*M_PI/D_CC*(double)j;
				glVertex3d(p.x()+r*cos(th)*cos(fa),p.z()+r*sin(fa),p.y()+r*sin(th)*cos(fa));
			}
		}
		glEnd();
		
	}


	class curve : public circle{
	protected:
	public:
		void dr(void);
	};

	void curve::dr(void){
		glColor3d(color.x(),color.y(),color.z());
		glBegin(GL_LINE_LOOP);
		int i = 0;
		double th;
		for(i=0;i<D_CC;i++){
			th = 2.0*M_PI/D_CC*(double)i;
			glVertex2d(p.x()+r*cos(th),p.y()+r*sin(th));
		}
		glEnd();
		
	}


	class curveZ :public circle{
		protected:
			Vector3d p;//p:位置，nvec:法線ベクトル
		public:
			void setp(const Vector3d &po);
			void dr(void);
	};

	void curveZ::setp(const Vector3d &po){
		p = po;
	}

	void curveZ::dr(void){
		glColor3d(color.x(),color.y(),color.z());
		glBegin(GL_LINE_LOOP);
		int i = 0;
		double th;
		for(i=0;i<D_CC;i++){
			th = 2.0*M_PI/D_CC*(double)i;
			glVertex3d(p.x()+r*cos(th),p.z(),p.y()+r*sin(th));
		}
		glEnd();
		
	}

	class arrow : public link{
	public:
		void	dr(void);
	};

	void arrow::dr(void){
		Vector2d mp;
		mp = (ep -sp)*3.0/4.0 + sp;
		glColor3d(color.x(),color.y(),color.z());
		glBegin(GL_POLYGON);
		double thett;
		thett = std::atan2(sp.y()-mp.y(),sp.x()-mp.x());
		glVertex2d(sp.x()-think/2*sin(thett),sp.y()+think/2*cos(thett));
		glVertex2d(sp.x()+think/2*sin(thett),sp.y()-think/2*cos(thett));
		glVertex2d(mp.x()+think/2*sin(thett),mp.y()-think/2*cos(thett));
		glVertex2d(mp.x()-think/2*sin(thett),mp.y()+think/2*cos(thett));
		glEnd();
		glBegin(GL_POLYGON);
		glVertex2d(mp.x()-think*sin(thett),mp.y()+think*cos(thett));
		glVertex2d(ep.x(),ep.y());
		glVertex2d(mp.x()+think*sin(thett),mp.y()-think*cos(thett));
		glEnd();
		
	}

	class locus{
	protected:
		int num;
		Vector2d pr;
		link locus[10000];
		Vector3d color;
	public:
		void set_v(const Vector2d &in,const int &n);
		void setcolor(const Vector3d &iro);
		void dr(void);
	};

	void locus::set_v(const Vector2d &in,const int &n){
		num = n;
		if(n>1){
			locus[n-1].setsp(pr);
			locus[n-1].setep(in);
			locus[n-1].setthink(0.005);
			locus[n-1].setcolor(color);
		}
		pr = in;
	}

	void locus::setcolor(const Vector3d &iro){
		color = iro;
	}
	
	void locus::dr(void){
		int a;
		if(num>1){
			for(a=0;a<num-1;a++){
				locus[a].dr();
			}
		}
	}
}