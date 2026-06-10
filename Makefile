CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra
TARGET = zigbee_reader

all: $(TARGET)

$(TARGET): zigbee_reader.cpp
	$(CXX) $(CXXFLAGS) -o $(TARGET) zigbee_reader.cpp

clean:
	rm -f $(TARGET)

.PHONY: all clean
