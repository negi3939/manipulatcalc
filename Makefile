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
	TARGET=ur3
endif

ifeq ($(TARGET),ur3)
	SOURCE_MAIN = ur3main.cpp
	SOURCE_SUB = mymath.cpp solvenu.cpp inversekinematics.cpp inversedynamics.cpp
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

DIRX = /usr/X11R6/lib

CXXFLAGS = -I ~/eigenlib/eigen-3.3.7/ -fpermissive
LDFLAGS	 = -L "$(DIRX)" -lglut -lGLU -lGL -lXmu  -lXext -lX11 -lm  -pthread -std=c++11

all: $(PROGRAM)

%.out: %.o $(SUBOBJ)
	g++ -o $@ $^ $(LDFLAGS) -w
	#$(COMMAND)
%.o : %.cpp
	g++ -o $@ $< -c $(CXXFLAGS) -w
clean:
	rm -f *.o $(PROGRAM)