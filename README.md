# Negi式IKIDソルバ
任意の軸のマニピュレータの運動学・運動力学ソルバ  
DHパラメータを入力して使う  
そのうちMMDのモーションとかにつかいたいなぁ  

# 数値的方程式解法 
mymath.cpp と　solvenu.cppを一緒に用いることで使える．ニュートン法を用いて一般方程式の解を得る．Funcveクラスを継承してfunctionをオーバーライドして使用する．

# Inverse kinetmatics
mymath.cpp と　solvenu.cppと inversekinematics.cppを一緒に用いることで使える．DHパラメータをinvkSolvenu::setdhparameter()で指定し，目標手先位置姿勢(クオータニオン)をinvkSolvenu::settargetfx()で設定する．回転行列からクオータニオンを得るには invkSolvenu::matrixtoquatanion()を用いる．invkSolvenu::getangle()で設定した手先位置姿勢となる関節角度(-π<θ<π)を得る．
copyメソッドを所有しており,このクラスを継承すればDHパラメータ等をまるごとコピーできる．

# Inverse Dynamics
mymath.cpp と　solvenu.cppと inversekinematics.cppとinversdynamics.cppを一緒に用いることで使える．DHパラメータはinvdSolvenu::setdhparameter()で指定することもinvdSolvenu::copy()でinverse kinematics用のインスタンスを与えてしてすることもできる．
目標手先速度角速度をinvdSolvenu::settargetfx()で設定する．invkSolvenu::getangvel()で設定した手先速度角速度となる関節角角度を得る．

# IDpy
python用のWrapper.$make target=idpy 実行後に import IDpyするとpythonで順運動力学と逆運動力学が使える．
今はCRANE用のDHパラメータが入っている.
 $ >> import IDpy as iD  
 $ >> cr = iD.Negi39FIKFID()  
 $ >> cr.setjointnum(7) #set jointnum  
 $ >> cr.setdhparameter((int) joint,(double_vector)[thetaoffset,a,d,alpha])# set DH parameter  
 $ >> jointangle = [0.1,0.2,0.3,0.4,0.5,0.6,0.7]       #angle rad  
 $ >> jointtau = [1.1,1.2,1.3,1.4,1.5,1.6,1.7]         #angle tau N*m  
 $ >> forcemoment = cr.getforce(jointangle,jointtau)   #get force of end effector   
 $ >> tau = cr.gettau(jointangle,forcemoment)          #get joint tau  


# Makefile
targetを指定するにはmake時にtargetに格納させて使う．また，基本的に実行入る作成後，そのまま実行させる使用にしているが，実行させたくない際はargv=0をつける．
実行ファイルに引数を付けたい場合はargv=hogeをつけると./hogehoge.out hogeのようにコマンド引数を渡せる．
projectを追加する際にはifeq()を用いてSOURCE_MAINとSOURCE_SUBにそれぞれファイルを指定する．
以下使用例
    $ make                     # You can get the executable file which written in TARGET. And the executable file will run.  
    $ make target=hoge         # You can get the executable file which written in hoge. And the executable file will run.       
    $ make argv=hoge           # You can get the executable file which written in TARGET. And the executable file will run with hoge.   
    $ make notrun=1            # You can get the executable file which written in TARGET. The executable file will not run.	  
    $ make clean               # The executable file which written in TARGET will removed.  
    $ make clean target=hoge   # The executable file which written in hoge will removed.      

