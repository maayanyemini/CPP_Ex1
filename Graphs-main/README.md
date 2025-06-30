# 📚 Graph Algorithms Project

This project implements various graph algorithms like BFS, DFS, Dijkstra, Prim, and Kruskal,
along with the necessary graph data structures, including Queue, minHeap, and unionFind. 
These algorithms are designed to work with an undirected weighted graph represented by adjacency lists.

---

## 🏗️ Project Structure

- `Graph.h/cpp` – Represents the graph using adjacency lists.
- `Algorithms.h/cpp` – Implements algorithms: BFS, DFS, Dijkstra, Prim, Kruskal.
- `structures.h/cpp` – Contains supporting data structures:
- `Queue` – For BFS traversal.
- `minHeap` – For Dijkstra, Prim, and Kruskal.
- `unionFind` – For Kruskal's cycle detection.
- `AlgoTest.cpp, StructTest.cpp, GraphTest.cpp`-Unit tests for algorithms,
data structures, and the graph implementation.

---

## Algorithm Demonstrator-main.cpp
This file contains example runs of all the graph algorithms implemented in the project.
It's used to showcase how each algorithm behaves on sample graphs.

## 🧪 Testing Coverage
This project includes unit tests for:
- Graph class operations
- Algorithms (BFS, DFS, Dijkstra,Prim,Kruskal)
- Data structures (Queue, minHeap, unionFind)

Tests are located in:
- `AlgoTest.cpp`
- `StructTest.cpp`
- `GraphTest.cpp`

### Run the tests with:
 make test


### 🔨 Compile the project
make Main

### ▶️ Run the main program (executes all algorithms)
./main

### 🧼 Check for memory leaks
make valgrind


## Notes
- All algorithms are implemented under the graph namespace.
- STL containers are intentionally not used.
- Designed to run on Linux environments (Ubuntu).

#### 👩‍💻 Author
- Eden Hassin
- Edenhassin@gmail.com
- Project completed: March–April 2025