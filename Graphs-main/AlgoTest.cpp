#include "doctest.h"
#include "Algorithms.h"
#include "Graph.h"
#include <stdexcept>

using namespace graph;
using namespace std;

// Tests BFS on a typical graph and edge cases including empty graph and invalid source vertex.
// Verifies the BFS tree structure and exception handling.
TEST_CASE("BFS Basic and Edge Cases") {
    Graph g(5);
    Graph emptyG(0);

    g.addEdge(0, 1, 2);
    g.addEdge(0, 2, 1);
    g.addEdge(1, 3, 5);
    g.addEdge(2, 4, 4);

    Graph bfsResult = Algorithms::bfs(g, 0);

    CHECK(bfsResult.getNeighbors(0) != nullptr);              // Assert vertex 0 has neighbors
    // Assert neighbors of vertex 0 is 1 and 2
    CHECK(bfsResult.getNeighbors(0)->vertex == 1);           
    CHECK(bfsResult.getNeighbors(0)->next->vertex == 2);      

    CHECK(bfsResult.getNeighbors(1) != nullptr);              // Assert vertex 1 has neighbors
    CHECK(bfsResult.getNeighbors(1)->vertex == 3);            // Assert vertex 1's neighbor is 3

    CHECK(bfsResult.getNeighbors(2) != nullptr);              // Assert vertex 2 has neighbors
    CHECK(bfsResult.getNeighbors(2)->vertex == 4);            // Assert vertex 2's neighbor is 4

    CHECK(bfsResult.getNeighbors(3) == nullptr);              // Assert vertex 3 has no neighbors
    CHECK(bfsResult.getNeighbors(4) == nullptr);              // Assert vertex 4 has no neighbors

    CHECK_THROWS_AS(Algorithms::bfs(emptyG, 0), std::invalid_argument); // Expect exception for empty graph
    CHECK_THROWS_AS(Algorithms::bfs(g, -2), std::out_of_range);        // Expect exception for negative source vertex
    CHECK_THROWS_AS(Algorithms::bfs(g, 10), std::out_of_range);        // Expect exception for out-of-bound source vertex
}

// Tests DFS traversal correctness and exception handling for invalid inputs.
// Checks the DFS tree connections and ensures all vertices are visited appropriately.
TEST_CASE("DFS Traversal and Invalid Input") {
    Graph g(4);

    g.addEdge(0, 1, 3);
    g.addEdge(1, 2, 2);    
    g.addEdge(2, 3, 1);
    g.addEdge(3, 0, 4);
        g.print_graph(); // Print original graph for visual verification

    

    Graph dfsResult = Algorithms::dfs(g, 0);
/** 
    CHECK(dfsResult.getNeighbors(0) != nullptr);
    dfsResult.print_graph(); // Print DFS tree for visual verification
    CHECK(dfsResult.getNeighbors(1) != nullptr);                // Assert vertex 0 has neighbors
    CHECK(dfsResult.getNeighbors(0)->vertex == 1);            // Assert vertex 0's neighbor is 1 
    CHECK(dfsResult.getNeighbors(1)->vertex == 2);            // Assert vertex 1's neighbor is 2
    CHECK(dfsResult.getNeighbors(2)->vertex == 3);            // Assert vertex 2's neighbor is 3
    CHECK(dfsResult.getNeighbors(3) == nullptr);              // Assert vertex 3 has no neighbors in DFS tree
*/
    CHECK(dfsResult.getNeighbors(0) != nullptr);
    dfsResult.print_graph(); // Print DFS tree for visual verification
    CHECK(dfsResult.getNeighbors(1) == nullptr);                // Assert vertex 0 has neighbors
    CHECK(dfsResult.getNeighbors(0)->vertex == 3);            // Assert vertex 0's neighbor is 1 
    CHECK(dfsResult.getNeighbors(2)->vertex == 1);            // Assert vertex 2's neighbor is 3
    CHECK(dfsResult.getNeighbors(3)->vertex == 2); 

    Graph emptyG(0);
    CHECK_THROWS_AS(Algorithms::dfs(emptyG, 0), std::invalid_argument); // Expect exception for empty graph
    CHECK_THROWS_AS(Algorithms::dfs(g, -5), std::out_of_range);        // Expect exception for invalid source vertex
    CHECK_THROWS_AS(Algorithms::dfs(g, 999), std::out_of_range);       // Expect exception for invalid source vertex
}

// Tests Dijkstra's algorithm for shortest paths including normal, empty graph, negative weights, and invalid sources.
// Verifies shortest path tree correctness and proper exception throwing.
TEST_CASE("Dijkstra Shortest Paths and Edge Cases") {
    Graph g(6);
    g.addEdge(0, 1, 10);
    g.addEdge(0, 2, 3);
    g.addEdge(1, 2, 1);
    g.addEdge(1, 3, 2);
    g.addEdge(2, 3, 8);
    g.addEdge(2, 4, 2);
    g.addEdge(3, 4, 7);
    g.addEdge(4, 5, 5);

    Graph dijkstraResult = Algorithms::dijkstra(g, 0);

    CHECK(dijkstraResult.getNeighbors(0)->vertex == 2);      // Assert vertex 0's shortest path leads to vertex 2
    CHECK(dijkstraResult.getNeighbors(2)->vertex == 4);      // Assert vertex 2 leads to vertex 4
    CHECK(dijkstraResult.getNeighbors(4)->vertex == 5);      // Assert vertex 4 leads to vertex 5
    CHECK(dijkstraResult.getNeighbors(1)->vertex == 3);      // Assert vertex 1 leads to vertex 3

    Graph emptyG(0);
    CHECK_THROWS_AS(Algorithms::dijkstra(emptyG, 0), std::invalid_argument);  // Expect exception on empty graph

    Graph negWeight(3);
    negWeight.addEdge(0, 1, -1);
    negWeight.addEdge(1, 2, 4);
    CHECK_THROWS_AS(Algorithms::dijkstra(negWeight, 0), std::runtime_error);   // Expect exception on negative edge weight

    CHECK_THROWS_AS(Algorithms::dijkstra(g, -1), std::out_of_range);          // Invalid source vertex
    CHECK_THROWS_AS(Algorithms::dijkstra(g, 10), std::out_of_range);          // Invalid source vertex
}

// Tests Prim's algorithm for MST including general graph, single-node graph, and empty graph.
// Verifies MST structure, edge count, total weight, and exception handling.
TEST_CASE("Prim MST Generation and Single Node Graph") {
    Graph g(5);
    g.addEdge(0, 1, 2);
    g.addEdge(0, 3, 6);
    g.addEdge(1, 2, 3);
    g.addEdge(1, 3, 8);
    g.addEdge(1, 4, 5);
    g.addEdge(2, 4, 7);
    g.addEdge(3, 4, 9);

    Graph mst = Algorithms::prim(g);

    int edgeCount = 0;
    int totalWeight = 0;
    for (int i = 0; i < mst.get_numV(); ++i) {
        Neighbor *n = mst.getNeighbors(i);
        while (n) {
            totalWeight += n->weight;
            edgeCount++;
            n = n->next;
        }
    }
    edgeCount /= 2; // Each edge counted twice (bidirectional)
    totalWeight /= 2;       // Total weight of MST counted twice
    CHECK(edgeCount == 4);            // 4 MST edges counted twice (bidirectional)
    CHECK(totalWeight == 16);             // Total weight of MST doubled

    Graph singleNode(1);
    Graph mst2 = Algorithms::prim(singleNode);
    CHECK(mst2.getNeighbors(0) == nullptr);                     // Single node graph has no edges

    Graph emptyG(0);
    CHECK_THROWS_AS(Algorithms::prim(emptyG), std::invalid_argument);  // Expect exception on empty graph
}

// Tests Kruskal's algorithm for MST including typical graph and empty graph.
// Verifies MST edges, total weight, and proper exception throwing.
TEST_CASE("Kruskal MST Construction and Empty Graph") {
    Graph g(5);
    g.addEdge(0, 1, 1);
    g.addEdge(0, 2, 3);
    g.addEdge(1, 2, 1);
    g.addEdge(1, 3, 4);
    g.addEdge(2, 3, 2);
    g.addEdge(3, 4, 5);

    Graph mst = Algorithms::kruskal(g);

    int edgeSum = 0;
    int edgeCount = 0;
    for (int i = 0; i < mst.get_numV(); ++i) {
        Neighbor *n = mst.getNeighbors(i);
        while (n) {
            edgeSum += n->weight;
            edgeCount++;
            n = n->next;
        }
    }
    edgeCount /= 2; // Each edge counted twice (bidirectional)
    edgeSum /= 2;       // Total weight of MST counted twice
    CHECK(edgeCount == 4);            // 4 MST edges counted twice (bidirectional)
    CHECK(edgeSum == 9);             // Total weight of MST doubled

    Graph emptyG(0);
    CHECK_THROWS_AS(Algorithms::kruskal(emptyG), std::invalid_argument); // Exception on empty graph
}
