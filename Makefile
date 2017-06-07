all: PathFinder

PathFinder:                 PathFinder.o
	g++ -Wall -g -std=c++0x PathFinder.o -o PathFinder

# core code
PathFinder.o: PathFinder.cxx
	g++ -Wall -g -std=c++0x -c PathFinder.cxx
