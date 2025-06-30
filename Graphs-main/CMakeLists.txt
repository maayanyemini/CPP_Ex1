cmake_minimum_required(VERSION 3.10)
project(GraphProject)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_executable(Exe1
    main.cpp
    Graph.cpp
    Graph.h
    structures.cpp
    structures.h
    Algorithms.cpp
    Algorithms.h
        AlgoTest.cpp
        GraphTest.cpp
        StructTest.cpp
)

