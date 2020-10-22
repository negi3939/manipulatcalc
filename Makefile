################################################################################################################################################
###        In this Makefile made by Negi3939, you can use as                                                               	                     ###
###        $ make                     # You can get the executable file which written in TARGET.                                             ###
###        $ make clean               # The executable file which written in TARGET will removed.                                            ###
################################################################################################################################################

TARGET=$(MAKECMDGOALS)
ifeq ($(MAKECMDGOALS),)
	TARGET=ik
endif
ifeq ($(MAKECMDGOALS),clean)
	TARGET=ik
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

ifeq ($(TARGET),idpy)
	SOURCE_MAIN = IDpy.cpp
	SOURCE_SUB = mymath.cpp solvenu.cpp inversekinematics.cpp inversedynamics.cpp
	CXXFLAGS += -I/usr/include/python2.7 -fPIC
endif

ifeq ($(TARGET),hoge)
	SOURCE_MAIN = hoge.cpp
	SOURCE_SUB = mymath.cpp 
	#animat.cpp
endif

PROGRAM = $(SOURCE_MAIN:%.cpp=%.out)
MAINOBJ = $(SOURCE_MAIN:%.cpp=%.o)
SUBOBJ = $(SOURCE_SUB:%.cpp=%.o)

#FOR PYTHON WRAPPER
IFILE = $(SOURCE_MAIN:%.cpp=%.i)
WRAPFILE = $(SOURCE_MAIN:%.cpp=%_wrap.cxx)
WRAPOBJ = $(SOURCE_MAIN:%.cpp=%_wrap.o)
SOFILE = $(SOURCE_MAIN:%.cpp=_%.so)
PYFILE = $(SOURCE_MAIN:%.cpp=%.py)

ik: $(PROGRAM)
id: $(PROGRAM)
ur3: $(PROGRAM)
all: $(PROGRAM)

idpy: $(MAINOBJ) $(SUBOBJ)
	g++ -shared -Wl,-soname,IDpy.so -o IDpy.so mymath.o solvenu.o inversekinematics.o inversedynamics.o IDpy.o -lpython2.7 -lboost_python

%.out: %.o $(SUBOBJ)
	g++ -o $@ $^ $(LDFLAGS) -w
	#$(COMMAND)
%.o : %.cpp
	g++ -o $@ $< -c $(CXXFLAGS) -w
clean:
	rm -f *.o *.so $(PROGRAM)
cleanall:
	rm -f *.o *.so *.out