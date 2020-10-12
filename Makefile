################################################################################################################################################
###        In this Makefile made by taka, you can use as                                                               	                     ###
###        $ make                     # You can get the executable file which written in TARGET. And the executable file will run.           ###
###        $ make target=hoge         # You can get the executable file which written in hoge. And the executable file will run.             ###
###        $ make argv=hoge           # You can get the executable file which written in TARGET. And the executable file will run with argv. ###
###        $ make notrun=1            # You can get the executable file which written in TARGET. The executable file will not run.		     ###
###        $ make clean               # The executable file which written in TARGET will removed.                                            ###
###        $ make clean target=hoge   # The executable file which written in hoge will removed.                                              ###
################################################################################################################################################

ifdef target
	TARGET=$(target)
else
	TARGET=ikpy
	#ik
	#ur3
endif


DIRX = /usr/X11R6/lib

CXXFLAGS = -I ~/eigenlib/eigen-3.3.7/ -fpermissive
LDFLAGS	 = -L "$(DIRX)" -lglut -lGLU -lGL -lXmu  -lXext -lX11 -lm  -pthread -std=c++11

ifeq ($(TARGET),ur3)
	SOURCE_MAIN = ur3main.cpp
	SOURCE_SUB = mymath.cpp solvenu.cpp inversekinematics.cpp inversedynamics.cpp
endif

ifeq ($(TARGET),ik)
	SOURCE_MAIN = inversekinematics.cpp
	SOURCE_SUB = mymath.cpp solvenu.cpp
	CXXFLAGS += -DIK_IS_MAIN
endif

ifeq ($(TARGET),id)
	SOURCE_MAIN = inversedynamics.cpp
	SOURCE_SUB = mymath.cpp solvenu.cpp inversekinematics.cpp
	CXXFLAGS += -DID_IS_MAIN
endif

ifeq ($(TARGET),hoge)
	SOURCE_MAIN = hoge.cpp
	SOURCE_SUB = mymath.cpp 
	#animat.cpp
endif

ifdef argv
	COMMAND = \rm *.o;echo run;./$(SOURCE_MAIN:%.cpp=%.out) argv
else
	COMMAND = \rm *.o;echo run;./$(SOURCE_MAIN:%.cpp=%.out)
endif
ifdef notrun
		COMMAND = \rm *.o;echo You got $(SOURCE_MAIN:%.cpp=%.out).
endif

PROGRAM = $(SOURCE_MAIN:%.cpp=%.out)
SUBOBJ = $(SOURCE_SUB:%.cpp=%.o)

ifeq ($(TARGET),ikpy)
CXXFLAGS += -I/usr/include/python2.7
ikpu.py:
	g++ -c -fPIC mymath.cpp -o mymath.o  $(CXXFLAGS) -w
	g++ -c -fPIC solvenu.cpp -o solvenu.o  $(CXXFLAGS) -w 
	g++ -c -fPIC inversekinematics.cpp -o inversekinematics.o  $(CXXFLAGS) -w
	g++ -c -fPIC inversedynamics.cpp -o inversedynamics.o  $(CXXFLAGS) -w
	g++ -c -fPIC IDpy.cpp -o IDpy.o  $(CXXFLAGS) -w
	g++ -shared -Wl,-soname,IDpy.so -o IDpy.so mymath.o solvenu.o inversekinematics.o inversedynamics.o IDpy.o -lpython2.7 -lboost_python
endif

#FOR PYTHON WRAPPER
IFILE = $(SOURCE_MAIN:%.cpp=%.i)
WRAPFILE = $(SOURCE_MAIN:%.cpp=%_wrap.cxx)
WRAPOBJ = $(SOURCE_MAIN:%.cpp=%_wrap.o)
SOFILE = $(SOURCE_MAIN:%.cpp=_%.so)
PYFILE = $(SOURCE_MAIN:%.cpp=%.py)

all: $(PROGRAM)

%.out: %.o $(SUBOBJ)
	g++ -o $@ $^ $(LDFLAGS) -w
	#$(COMMAND)
%.o : %.cpp
	g++ -o $@ $< -c $(CXXFLAGS) -w
clean:
	rm -f *.o *.so $(PROGRAM)