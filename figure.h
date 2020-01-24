/*figure.h*/
/*図形描写用ヘッダ*/
#ifndef FIGURE_H
#define FIGURE_H

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

	class line3d :public line{
		private:
			Vector3d color;
			Vector3d sp;
			Vector3d ep;
		public:
			line3d();
			void setcolor(const Vector3d &iro);
			void setsp(const Vector3d &s);
			void setep(const Vector3d &e);
			void setth(const double &w);
			void dr(void);
	};
	
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
	
#define D_C   (16) //bunkarusuu
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

	class circle3d : public circle{
		protected:
			Vector3d p;
		public:
			void setp(const Vector3d &po);
			void dr(void);
	};

	class curve : public circle{
		protected:
		public:
			void dr(void);
	};

	class curveZ :public circle{
		protected:
			Vector3d color;
			Vector3d p;//p:位置，nvec:法線ベクトル
		public:
			void setp(const Vector3d &po);
			void dr(void);
	};

	class arrow : public link{
		public:
			void dr(void);
	};

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

}


#endif