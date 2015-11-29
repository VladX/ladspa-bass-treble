all:
	clang++-3.6 -O3 -march=native -mtune=native -std=c++11 -Wall -shared -fPIC basstreble.cpp -o basstreble.so
clean:
	rm basstreble.so