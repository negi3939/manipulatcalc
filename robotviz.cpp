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

class Robotviz : public Animation{
    protected:
        void setdispf();//displayの関数を設定
    public:
        Robotviz();
    protected:
    static void display(){
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            float anglex = 0.0f;
            glViewport(0, 0, 320.0d, 240.0d);
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            //視野角,アスペクト比(ウィンドウの幅/高さ),描画する範囲(最も近い距離,最も遠い距離)
            gluPerspective(30.0, 320.0d / 240.0d, 1.0, 1000.0);
            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();
            //視点の設定
            gluLookAt(500.0,500.0,-500.0, //カメラの座標
            5.0,5.0,5.0, // 注視点の座標
            1.0,0.0,0.0); // 画面の上方向を指すベクトル
            //ライトの設定
            GLfloat lightpos[] = { 20.0, 15.0, -50.0, 0.0 };
            GLfloat Blue[] = { 1.0, 0.0, 0.0, 0.5 };
            //glLightfv(GL_LIGHT0, GL_POSITION, lightpos);
            //マテリアルの設定
            glMaterialfv(GL_FRONT, GL_DIFFUSE, Blue);
            //回転
            glRotatef(anglex,1.0f,0.0f,0.0f);//X軸を回転
            draw::Cylinder joint;
            joint.setp(30.0,50.0,10);
            joint.dr();
            //glutSwapBuffers();
    }
            
};
Robotviz::Robotviz(){
    setdispf();
}

void Robotviz::setdispf(){
    disp = Robotviz::display;
}

#if defined(VIZ_IS_MAIN)
int main(){
    Robotviz rv;
    rv.start();
    while(1){}
}
#endif