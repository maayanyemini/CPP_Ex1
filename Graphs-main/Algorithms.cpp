// Algorithms.cpp
using namespace std;
#include "Algorithms.h"
#include "Graph.h"
#include <iostream>
#include "structures.h"
#include <climits>


namespace graph {

/**
 * Performs Breadth-First Search (BFS) on a graph starting from a given vertex.
 * @param g: The input graph.
 * @param srcVertex: The starting vertex for BFS.
 * @return A BFS tree (Graph object) that represents the traversal.
 * @throws std::out_of_range if srcVertex is out of bounds.
 * @throws std::invalid_argument if the graph is empty.
 */
  Graph Algorithms::bfs(const Graph &g, int srcVertex) {
    int vertexCount = g.get_numV();

    // Check for empty graph
    if (vertexCount == 0) {
        throw std::invalid_argument("Graph is empty");
    }

    // Check if source vertex is valid
    if (srcVertex < 0 || srcVertex >= vertexCount) {
        throw std::out_of_range("Source vertex out of bounds");
    }

    // Create a new graph to represent the BFS traversal tree
    Graph bfsTree(vertexCount);  // This is a constructor call to create an empty graph with 'vertexCount' vertices

    // Allocate and initialize visited array
    bool *visited = new bool[vertexCount];
    for (int i = 0; i < vertexCount; ++i) {
        visited[i] = false;  // Initially, no vertices are visited
    }

    visited[srcVertex] = true;  // Mark the source vertex as visited
    Queue queue;                // Create a queue for BFS traversal
    queue.enqueue(srcVertex);   // Enqueue the source vertex to start BFS
    // Traverse the graph level by level using a queue
    while (!queue.isEmpty()) {
        int currentVertex = queue.dequeue();

        // Get all neighbors of the current vertex
        Neighbor *neighbor = g.getNeighbors(currentVertex);
        while (neighbor) {
            int neighborVertex = neighbor->vertex;

            // If neighbor hasn't been visited, enqueue it and add the edge to the BFS tree
            if (!visited[neighborVertex]) {
                visited[neighborVertex] = true;
                queue.enqueue(neighborVertex);
                bfsTree.addNeighbor(currentVertex, neighborVertex, neighbor->weight);
            }

            neighbor = neighbor->next;
        }
    }

    delete[] visited;
    return bfsTree;
}
    /**
     * Helper function for recursive Depth-First Search (DFS).
     * @param g: The input graph.
     * @param srcVertex: Current vertex being visited.
     * @param dfsTree: The DFS tree being constructed.
     * @param visited: Array tracking visited vertices.
     */
        void Algorithms::recDfs(const Graph &g, int srcVertex, Graph &dfsTree, bool *visited) {
        visited[srcVertex] = true;

        // Get all neighbors of the current vertex
        Neighbor *neighbors = g.getNeighbors(srcVertex);
        while (neighbors) {
            int vertex = neighbors->vertex;

            // Visit unvisited neighbors recursively and add them to DFS tree
            if (!visited[vertex]) {
                dfsTree.addNeighbor(srcVertex, vertex, neighbors->weight);
                recDfs(g, vertex, dfsTree, visited);
            }
            neighbors = neighbors->next;
        }
    }

/**
 * Performs Depth-First Search (DFS) on a graph starting from a given vertex.
 * @param g: The input graph.
 * @param srcVertex: The starting vertex for DFS.
 * @return A DFS tree (Graph object) that represents the traversal.
 * @throws std::invalid_argument if the graph is empty.
 * @throws std::out_of_range if srcVertex is out of bounds.
 */
Graph Algorithms::dfs(const Graph &g, int srcVertex) {
    int vertexCount = g.get_numV();
      if (vertexCount == 0) {
        throw std::invalid_argument("Graph is empty");
    }
    if (srcVertex < 0 || srcVertex >= vertexCount) {
        throw std::out_of_range("Source vertex: out of bounds");
    }

    bool *visited = new bool[vertexCount]();
    Graph dfsResult(vertexCount);

    // Visit the connected component starting from srcVertex
    recDfs(g, srcVertex, dfsResult, visited);

    // Ensure that all disconnected components are visited as well
    for (int v = 0; v < vertexCount; ++v) {
        if (!visited[v]) {
            recDfs(g, v, dfsResult, visited);
        }
    }

    delete[] visited;
    return dfsResult;
}

   /**
     * Performs Dijkstra's algorithm to find shortest paths from a source vertex.
     * @param g: The input graph.
     * @param srcVertex: The source vertex.
     * @return A tree (Graph object) representing shortest paths.
     * @throws std::out_of_range if srcVertex is out of bounds.
     * @throws std::invalid_argument if the graph is empty.
     * @throws std::runtime_error if a negative weight is detected.
     */
   Graph Algorithms::dijkstra(const Graph &g, int srcVertex) {
    const int vertexCount = g.get_numV();

    // Check for empty graph
    if (vertexCount == 0) {
        throw std::invalid_argument("Graph is empty");
    }

    // Check if source vertex is valid
    if (srcVertex < 0 || srcVertex >= vertexCount) {
        throw std::out_of_range("Source vertex out of bounds");
    }

    const int INF = INT_MAX;
    Graph result(vertexCount);  // Output graph to represent shortest-path tree

    // Initialize distance, visited, and parent arrays
    int *dist = new int[vertexCount];
    bool *visited = new bool[vertexCount];
    int *parent = new int[vertexCount];

    for (int i = 0; i < vertexCount; i++) {
        dist[i] = INF;          // Set all distances to "infinity"
        visited[i] = false;     // No vertex has been visited yet
        parent[i] = -1;         // No parents initially
    }

    dist[srcVertex] = 0;        // Distance to source is zero

    minHeap heap;
    heap.insert(srcVertex, srcVertex, 0);  // Insert source vertex into heap

    // Main Dijkstra loop
    while (!heap.isEmpty()) {
        edge e = heap.extractMin();  // Get the edge with minimum weight
        int u = e.dstV;

        if (visited[u]) continue;   // Skip already visited vertices

        visited[u] = true;          // Mark current vertex as visited

        Neighbor *n = g.getNeighbors(u);  // Traverse all neighbors
        while (n) {
            // Dijkstra does not allow negative weights
            if (n->weight < 0) {
                delete[] dist;
                delete[] visited;
                delete[] parent;
                throw std::runtime_error("Negative weight detected!");
            }

            // Try to relax edge (u, n->vertex)
            relax(u, n->vertex, n->weight, dist, heap, parent);
            n = n->next;
        }
    }

    // Build the shortest path tree based on parent array
    for (int i = 0; i < vertexCount; i++) {
        if (parent[i] != -1) {
            // Add edge from parent[i] to i with correct weight
            result.addNeighbor(parent[i], i, dist[i] - dist[parent[i]]);
        }
    }

    // Free dynamic memory
    delete[] dist;
    delete[] visited;
    delete[] parent;

    return result;
}

    /**
     * Performs the relaxation step in Dijkstra's algorithm.
     * @param u: Current vertex.
     * @param vertex: Neighbor vertex.
     * @param weight: Edge weight.
     * @param d: Distance array.
     * @param p: Min-heap used for selecting minimum distance vertex.
     * @param parent: Parent array used to construct shortest-path tree.
     */
    void Algorithms::relax(int u, int vertex, int weight, int *d, minHeap &p, int *parent) {
        // If a shorter path is found to 'vertex', update it
        if (d[u] + weight < d[vertex]) {
            d[vertex] = d[u] + weight;
            p.insert(u, vertex, weight);
            parent[vertex] = u;
        }
    }

    /**
     * Computes the Minimum Spanning Tree (MST) of a graph using Prim's algorithm.
     * @param g: The input graph.
     * @return A graph representing the MST.
     * @throws std::invalid_argument if the graph is empty.
     */
   Graph Algorithms::prim(const Graph &g) {
    const int vertexCount = g.get_numV();

    // Check if the graph is empty
    if (vertexCount == 0) {
        throw std::invalid_argument("Graph is empty");
    }

    // Array to track which vertices are included in the MST
    bool *inMST = new bool[vertexCount]();

    // Create a new graph to hold the MST result
    Graph mst(vertexCount);

    // Min-heap to choose the minimum weight edge at each step
    minHeap heap;

    // Start from vertex 0
    int start = 0;

    // Insert all edges from the starting vertex into the heap
    Neighbor *neighbors = g.getNeighbors(start);
    while (neighbors) {
        heap.insert(start, neighbors->vertex, neighbors->weight);
        neighbors = neighbors->next;
    }

    // Mark the starting vertex as included in the MST
    inMST[start] = true;

    // Process the heap until all vertices are included
    while (!heap.isEmpty()) {
        edge e = heap.extractMin();

        // If destination vertex is not yet in the MST
        if (!inMST[e.dstV]) {
            inMST[e.dstV] = true;

            // Add this edge to the MST result graph
            mst.addEdge(e.srcV, e.dstV, e.weight);

            // Insert all edges from the newly added vertex
            Neighbor *n = g.getNeighbors(e.dstV);
            while (n) {
                if (!inMST[n->vertex]) {
                    heap.insert(e.dstV, n->vertex, n->weight);
                }
                n = n->next;
            }
        }
    }

    // Clean up and return the MST
    delete[] inMST;
    return mst;
}

/**
     * Computes the Minimum Spanning Tree (MST) of a graph using Kruskal's algorithm.
     * @param g: The input graph.
     * @return A graph representing the MST.
     * @throws std::invalid_argument if the graph is empty.
     */
   Graph Algorithms::kruskal(const Graph &g) {
    int vertexCount = g.get_numV();

    // Check if the graph is empty
    if (vertexCount == 0) {
        throw std::invalid_argument("Graph is empty.");
    }

    // Min-heap to store all edges in the graph by weight
    minHeap edgeHeap;

    // Union-Find data structure to detect cycles
    unionFind uf;

    // Graph to store the MST result
    Graph mst(vertexCount);

    // Add all edges to the heap (only once per undirected edge)
    for (int i = 0; i < vertexCount; ++i) {
        Neighbor *n = g.getNeighbors(i);
        while (n) {
            // Avoid inserting both (i,j) and (j,i) in undirected graph
            if (i < n->vertex) {
                edgeHeap.insert(i, n->vertex, n->weight);
            }
            n = n->next;
        }
    }

    // While there are edges in the heap
    while (!edgeHeap.isEmpty()) {
        edge e = edgeHeap.extractMin();

        // Check if adding this edge will form a cycle
        if (uf.find(e.srcV) != uf.find(e.dstV)) {
            // If not, include the edge in the MST
            mst.addEdge(e.srcV, e.dstV, e.weight);
            uf.unionSet(e.srcV, e.dstV);  // Merge the sets
        }
    }

    return mst;
} // namespace graph

}