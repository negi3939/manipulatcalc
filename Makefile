################################################################################################################################################
###        In this Makefile made by Negi3939, you can use as                                                          	                     ###
###        $ make ik                   # You can get the executable file which can solve IK.                                                 ###
###        $ make id                   # You can get the executable file which can solve ID.                                                 ###
###        $ make idpy                 # You can get the python library.                                                                     ###
###        $ make clean               # The executable file which written in TARGET will removed.                                            ###
################################################################################################################################################

TARGET=$(MAKECMDGOALS)
ifeq ($(MAKECMDGOALS),)
	TARGET=negi
endif
ifeq ($(MAKECMDGOALS),clean)
	TARGET=negi
endif

DIRX = /usr/X11R6/lib
CXXFLAGS = -I ~/eigenlib/eigen-3.3.7/ -fpermissive
LDFLAGS	 = -L "$(DIRX)" -lm  -pthread -std=c++11

GCCVERSION = $(shell g++ --version | grep ^g++)
ifeq "$(GCCVERSION)" "g++ (GCC) 3.3.5 (Debian 1:3.3.5-13)"
	CXXFLAGS += -DGCC3p3
endif

ifeq ($(TARGET),ur3)
	SOURCE_MAIN = ur3main.cpp
	SOURCE_SUB = mymath.cpp solvenu.cpp inversekinematics.cpp inversedynamics.cpp
endif

ifeq ($(TARGET),negi)
	SOURCE_MAIN = Negi39IKID.cpp
	SOURCE_SUB = mymath.cpp solvenu.cpp inversekinematics.cpp inversedynamics.cpp
	CXXFLAGS += -DNEGI_IS_MAIN
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
	LDFLAGS += -lpython2.7 -lboost_python 
endif

ifeq ($(TARGET),hoge)
	SOURCE_MAIN = hoge.cpp
	SOURCE_SUB = mymath.cpp 
	#animat.cpp
endif

PROGRAM = $(SOURCE_MAIN:%.cpp=%.out)
MAINOBJ = $(SOURCE_MAIN:%.cpp=%.o)
SUBOBJ = $(SOURCE_SUB:%.cpp=%.o)
SOFILE = $(SOURCE_MAIN:%.cpp=%.so)


negi: $(PROGRAM)
ik: $(PROGRAM)
id: $(PROGRAM)
ur3: $(PROGRAM)
idpy: $(MAINOBJ) $(SUBOBJ)
	g++ -shared -Wl,-soname,$(SOFILE) -o $(SOFILE) $^ $(LDFLAGS) -w

%.out: %.o $(SUBOBJ)
	g++ -o $@ $^ $(LDFLAGS) -w
%.o : %.cpp
	g++ -o $@ $< -c $(CXXFLAGS) -w
clean:
	rm -f *.o *.so $(PROGRAM)
cleanall:
	rm -f *.o *.so *.out