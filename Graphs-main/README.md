📊 Graph Algorithms – C++ Project
This project presents a comprehensive implementation of fundamental graph algorithms, including BFS, DFS, Dijkstra, Prim, and Kruskal, alongside the core data structures required for their execution: Queue, Min Heap, and Union-Find. The graph is modeled as an undirected, weighted structure using adjacency lists.

🧱 Project Layout
Graph.h / Graph.cpp – Defines the structure of the graph using adjacency lists.
Algorithms.h / Algorithms.cpp – Contains the logic for all the implemented graph traversal and pathfinding algorithms.
structures.h / structures.cpp – Houses the core utility data structures:
Queue: used in BFS.
minHeap: utilized in Dijkstra, Prim, and Kruskal algorithms.
unionFind: supports cycle detection in Kruskal.
AlgoTest.cpp, StructTest.cpp, GraphTest.cpp – Unit testing files for the algorithms, supporting structures, and the graph class.
Algorithm Demonstrator – main.cpp
A demonstration file that runs each algorithm on example graphs to illustrate how they function in practice. 

✅ Test Coverage

The project includes unit tests for:
Core operations of the graph class
All algorithm implementations: BFS, DFS, Dijkstra, Prim, Kruskal
Supporting structures: Queue, MinHeap, Union-Find
Tests are implemented in:
AlgoTest.cpp
StructTest.cpp
GraphTest.cpp
---



To run all tests run in bash:
make test
🛠️ Build Instructions
To compile the full project, run in bash:
make Main
To execute the main demo program:
./main

🧹 Memory Management
To check for memory leaks using Valgrind:
make valgrind

ℹ️ Notes
All algorithm implementations are encapsulated within the graph namespace.
The project avoids usage of the C++ STL containers intentionally for educational purposes.
Designed and tested for use in Linux environments (e.g., Ubuntu).


👩‍💻 Author
- Maayan Cohen Yemini
- Maayanyemini123@gmail.com
