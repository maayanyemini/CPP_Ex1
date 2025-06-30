// Graph.cpp
using namespace std;
#include <cstdio>
#include "Graph.h"
#include <iostream>


using namespace graph; 
    /**
     * Constructor for the Graph class.
     * Initializes an empty graph with the given number of vertices.
     * @param vertices: The number of vertices in the graph.
     * @throws std::invalid_argument if vertices is negative.
     */

    Graph::Graph(int vertices) : numV(vertices) {
    if (vertices < 0) {
        throw std::invalid_argument("vertices must be greater than zero");
    }

    // Allocate space for adjacency list
    adjacencyList = new Neighbor *[vertices]();
}

    Graph::Graph(const Graph &g) : numV(g.numV) {
    // Allocate space for adjacency list
    adjacencyList = new Neighbor *[numV]();
    
    // Deep copy: for safety you might want to actually copy the linked lists
    for (int i = 0; i < numV; i++) {
        Neighbor *current = g.adjacencyList[i];
        while(current){
        this->addEdge(i, current->vertex, current->weight);
        }
    }
}

    void Graph::operator=(const Graph &g) {
    
    for(int i = 0; i < numV; i++) {
        Neighbor *current = adjacencyList[i];
        // Delete linked list of neighbors
        while (current) {
            Neighbor *temp = current;
            current = current->next;
            delete temp;
        }
        adjacencyList[i] = nullptr;
    }
        // Allocate space for adjacency list
    adjacencyList = new Neighbor *[numV]();
    
    // Deep copy: for safety you might want to actually copy the linked lists
    for (int i = 0; i < numV; i++) {
        Neighbor *current = g.adjacencyList[i];
        while(current){
        this->addEdge(i, current->vertex, current->weight);
        }
    }
    }

    /**
     * Destructor for the Graph class.
     * Frees all dynamically allocated memory in the adjacency list.
     */
    Graph::~Graph() {
        for (int i = 0; i < numV; i++) {
            Neighbor *current = adjacencyList[i];
            // Delete linked list of neighbors
            while (current) {
                Neighbor *temp = current;
                current = current->next;
                delete temp;
            }
            adjacencyList[i] = nullptr;
        }
        delete[] adjacencyList;
    }

    /**
     * Adds an undirected edge between two vertices.
     * @param src: The source vertex.
     * @param dst: The destination vertex.
     * @param weight: The weight of the edge.
     * @throws std::runtime_error if the graph is empty or the edge already exists.
     * @throws std::invalid_argument if vertex indices are out of bounds.
     */
    void Graph::addEdge(int src, int dst, int weight) {
        if (numV == 0) {
            throw std::runtime_error("Cannot add an edge to an empty graph");
        }
        if (src < 0 || dst < 0 || src >= numV || dst >= numV) {
            throw std::invalid_argument("Invalid vertex index");
        }
        if (edgeExists(src, dst) || edgeExists(dst, src)) {
            throw std::runtime_error("Error: the edge already exists");
        }
        addNeighbor(src, dst, weight);
        addNeighbor(dst, src, weight);
    }

    /**
     * Checks if an edge exists between two vertices.
     * @param src: The source vertex.
     * @param dst: The destination vertex.
     * @return true if edge exists, false otherwise.
     */
    bool Graph::edgeExists(const int src, const int dst) const {
        const Neighbor *current = adjacencyList[src];
        while (current) {
            if (current->vertex == dst) {
                return true;
            }
            current = current->next;
        }
        return false;
    }

    /**
     * Adds a directed edge from src to dst.
     * @param src: The source vertex.
     * @param dst: The destination vertex.
     * @param weight: The weight of the edge.
     * @throws std::invalid_argument if vertex indices are out of bounds.
     * @throws std::runtime_error if the edge already exists.
     */
    void Graph::addNeighbor(int src, int dst, int weight) {
    if (src < 0 || dst < 0 || src >= numV || dst >= numV) {
        throw std::invalid_argument("Vertex's Index is out of bounds");
    }
    if (edgeExists(src, dst)) {
        throw std::runtime_error("Error: the edge already exists");
    }

    Neighbor *newNeighbor = new Neighbor{dst, weight, adjacencyList[src]};
    adjacencyList[src] = newNeighbor;
}


    /**
     * Removes an undirected edge between two vertices.
     * @param src: One endpoint.
     * @param dst: The other endpoint.
     */
    void Graph::removeEdge(int src, int dst) {
        removeEdgeFromTo(src, dst);
        removeEdgeFromTo(dst, src);
    }

    /**
     * Removes a directed edge from src to dst.
     * @param src: The source vertex.
     * @param dst: The destination vertex.
     * @throws std::runtime_error if the edge does not exist.
     */
    void Graph::removeEdgeFromTo(int src, int dst) {
        if (!edgeExists(src, dst)) {
            throw std::runtime_error("Error:Edge not exists");
        }
        Neighbor *current = adjacencyList[src];
        Neighbor *previous = nullptr;

        // Traverse the linked list to find and remove the edge
        while (current) {
            if (current->vertex == dst) {
                if (previous == nullptr) {
                    adjacencyList[src] = current->next;
                } else {
                    previous->next = current->next;
                }
                delete current;
                return;
            }
            previous = current;
            current = current->next;
        }
    }

    /**
     * Prints the adjacency list of the graph.
     */
    void Graph::print_graph() const {
        if (numV == 0 || adjacencyList == nullptr) {
            cout << "Empty graph." << endl;
            return;
        }

        for (int i = 0; i < numV; i++) {
            cout << "Vertex " << i << ": ";
            Neighbor *current = adjacencyList[i];
            if (current == nullptr) {
                cout << "No neighbors";
            }
            while (current) {
                cout << "<v:" << current->vertex << ",w:" << current->weight << "> ";
                current = current->next;
            }
            cout << endl;
        }
        cout << "\n" << endl;
    }

    /**
     * @return The number of vertices in the graph.
     */
    int Graph::get_numV() const {
        return numV;
    }

    /**
     * Returns the head of the neighbor list for a given vertex.
     * @param vertex: The vertex to get neighbors for.
     * @return Pointer to the first Neighbor.
     */
    Neighbor *Graph::getNeighbors(const int vertex) const {
        return adjacencyList[vertex];
    }
// namespace graph
