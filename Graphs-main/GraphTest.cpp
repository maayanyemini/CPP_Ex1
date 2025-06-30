#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "Graph.h"
#include <stdexcept>

using namespace graph;
using namespace std;

// Test creation of valid and invalid graphs
TEST_CASE("Graph Construction Validity") {
    CHECK_NOTHROW(Graph(0)); // Should allow empty graph
    CHECK_NOTHROW(Graph(4)); // Should allow graph with positive number of vertices
    CHECK_THROWS_AS(Graph(-1), std::invalid_argument); // Negative vertices should throw
}

// Test adding and removing edges (undirected and directed separately)
TEST_CASE("Edge Addition and Removal") {
    Graph undirected(3);
    Graph directed(3);

    // Adding edge to an empty graph should throw
    Graph empty(0);
    CHECK_THROWS_AS(empty.addEdge(0, 1, 7), std::runtime_error);

    // Add undirected edges (bidirectional)
    CHECK_NOTHROW(undirected.addEdge(0, 1, 10)); // Valid edge
    CHECK_NOTHROW(undirected.addEdge(0, 2, 5));  // Another valid edge
    CHECK_NOTHROW(undirected.addEdge(1, 2, 3));  // Third valid edge

    // Add directed edges
    CHECK_NOTHROW(directed.addNeighbor(0, 1, 4)); // 0 -> 1
    CHECK_NOTHROW(directed.addNeighbor(0, 2, 2)); // 0 -> 2
    CHECK_NOTHROW(directed.addNeighbor(1, 2, 1)); // 1 -> 2

    // Check if edges exist in undirected graph
    CHECK(undirected.edgeExists(0, 1)); // Edge 0 <-> 1 should exist
    CHECK(undirected.edgeExists(1, 0)); // Both directions
    CHECK(undirected.edgeExists(2, 1)); // Edge 2 <-> 1

    // Check directed behavior
    CHECK(directed.edgeExists(0, 1));   // 0 -> 1 exists
    CHECK_FALSE(directed.edgeExists(1, 0)); // 1 -> 0 does not exist
    CHECK(directed.edgeExists(0, 2));   // 0 -> 2 exists
    CHECK_FALSE(directed.edgeExists(2, 0)); // 2 -> 0 does not exist
    CHECK(directed.edgeExists(1, 2));   // 1 -> 2 exists
    CHECK_FALSE(directed.edgeExists(2, 1)); // 2 -> 1 does not exist

    // Remove an edge and verify
    CHECK_NOTHROW(undirected.removeEdge(0, 1)); // Remove 0 <-> 1
    CHECK_FALSE(undirected.edgeExists(0, 1));   // Should be gone
    CHECK_FALSE(undirected.edgeExists(1, 0));   // Both directions removed

    // Try removing same edge again (should throw)
    CHECK_THROWS_AS(undirected.removeEdge(0, 1), std::runtime_error);

    // Try adding edge with out-of-range index
    CHECK_THROWS_AS(undirected.addEdge(5, 0, 8), std::invalid_argument);
    CHECK_THROWS_AS(undirected.addEdge(1, -1, 2), std::invalid_argument);

    // Try adding duplicate edge
    CHECK_THROWS_AS(undirected.addEdge(0, 2, 5), std::runtime_error); // Already exists

} 
