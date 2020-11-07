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

#include"figure.h"
#include"animat.h"

namespace draw{
	
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

	void Cylinder::setp(double l_radius,double l_height,int l_sides){
		radius = l_radius;
		height = l_height;
		sides = l_sides;
	}	

	void Cylinder::setcolor(const Vector3d &iro){
		color = iro;
	}

	void Cylinder::dr(){
 		//上面
 		glNormal3d(0.0, 1.0, 0.0);
 		glBegin(GL_POLYGON);
 		for(double ii = 0; ii < sides; ii++) {
  			double t = M_PI*2/sides * (double)ii;
  			glVertex3d(radius * cos(t), height, radius * sin(t));
 		}
 		glEnd();
 		//側面
 		glBegin(GL_QUAD_STRIP);
 		for(double i=0;i<=sides;i=i+1){
  			double t = i*2*M_PI/sides;
  			glNormal3d(cos(t),0.0,sin(t));
  			glVertex3d((radius*cos(t)),-height,(radius*sin(t)));
  			glVertex3d((radius*cos(t)),height,(radius*sin(t)));
 		}
 		glEnd();
 		//下面
 		glNormal3d(0.0, -1.0, 0.0);
 		glBegin(GL_POLYGON);
 		for(double ii = sides; ii >= 0; --ii) {
  			double t = M_PI*2/sides * (double)ii;
  			glVertex3d(radius * cos(t), -height, radius * sin(t));
 		}
 		glEnd();
	}

}