CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2
TARGET = main

all: $(TARGET)

$(TARGET): main.cpp
	$(CXX) $(CXXFLAGS) -o $(TARGET) main.cpp -lrt

clean:
	rm -f $(TARGET)

.PHONY: all clean

