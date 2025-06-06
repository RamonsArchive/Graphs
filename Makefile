CXX=g++
CXXFLAGS?=-Wall -Wuninitialized -pedantic -Werror -g -O0 -std=c++17
OUTFILES=GraphTest

all: $(OUTFILES)

GraphTest: src/GraphTest.cpp src/Graph.cpp src/Graph.h src/Compare.h
	$(CXX) $(CXXFLAGS) -o GraphTest src/GraphTest.cpp src/Graph.cpp

gprof: src/GraphTest.cpp src/Graph.cpp src/Graph.h src/Compare.h
	make clean
	$(CXX) $(CXXFLAGS) -pg -o GraphTest src/GraphTest.cpp src/Graph.cpp

clean:
	$(RM) $(OUTFILES) *.o
