# solver for manipulator
Inverse kinetmaticとReverse dynamicsを計算　数値的に求める

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

