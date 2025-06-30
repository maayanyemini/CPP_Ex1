# Email:Edenhassin@gmail.com

CXX = g++
CXXFLAGS = -Wall -std=c++17 -g

TARGET = main
TEST_TARGET = test

SRCS = main.cpp Graph.cpp Algorithms.cpp structures.cpp
TEST_SRCS = StructTest.cpp GraphTest.cpp AlgoTest.cpp Graph.cpp Algorithms.cpp structures.cpp

OBJS = $(SRCS:.cpp=.o)
TEST_OBJS = $(TEST_SRCS:.cpp=.o)

# Default target to build everything (both main and test)
all: $(TARGET) $(TEST_TARGET)

# Target to build the main program
main: $(OBJS) 
	$(CXX) $(OBJS) -o $(TARGET) $(CXXFLAGS)

# Target to build the test program
test: $(TEST_OBJS)
	$(CXX) $(TEST_OBJS) -o $(TEST_TARGET)
	./$(TEST_TARGET)  # Run the test program after building

# Compile .cpp files into .o files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Target to run valgrind on the main program
valgrind: Main
	valgrind --leak-check=full --show-leak-kinds=all ./$(TARGET)


# Clean up generated files
clean:
	rm -f $(OBJS) $(TEST_OBJS) $(TARGET) $(TEST_TARGET)

.PHONY: all clean Main test valgrind valgrind-test
