src = *.cpp

all:
	g++ -g $(src)  -std=c++11 -O0 -o raytracer -lpthread
