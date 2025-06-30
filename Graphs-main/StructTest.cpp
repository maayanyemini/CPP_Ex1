#include "doctest.h"
#include "structures.h"

using namespace graph;
using namespace std;

TEST_CASE("Queue basic operations") {
    Queue q;

    // Verify that a newly created queue is empty
    CHECK(q.isEmpty());

    // Add an item and ensure the queue is no longer empty
    q.enqueue(2);
    CHECK(!q.isEmpty());

    // Remove the item and check it matches what was inserted
    int val = q.dequeue();
    CHECK(val == 2);

    // Queue should be empty again after removal
    CHECK(q.isEmpty());

    SUBCASE("Dequeue from an empty queue should return -1") {
        Queue q1;
        CHECK(q1.dequeue() == -1);
    }

    SUBCASE("Attempt to enqueue into a full queue should not alter contents") {
        Queue q2;
        // Fill the queue to maximum capacity
        for (int i = 0; i < MAX_SIZE - 1; ++i) {
            q2.enqueue(i);
        }
        // Attempt to enqueue beyond capacity
        q2.enqueue(102);
        // Verify the queue remains consistent
        CHECK(q2.dequeue() == 0);
    }
}

TEST_CASE("minHeap insertion and extraction order") {
    minHeap h;

    // Insert multiple edges with different weights
    h.insert(0, 1, 5);
    h.insert(1, 2, 3);
    h.insert(2, 3, 7);

    // Ensure extractMin gives smallest weight first
    edge e = h.extractMin();
    CHECK(e.weight == 3);

    // Extract the next smallest
    e = h.extractMin();
    CHECK(e.weight == 5);

    // Final extraction should be the largest
    e = h.extractMin();
    CHECK(e.weight == 7);
}

TEST_CASE("minHeap edge cases") {
    minHeap h;

    // New heap should be empty
    CHECK(h.isEmpty());

    // Extracting from empty heap should yield dummy edge
    edge e = h.extractMin();
    CHECK(e.dstV == -1);
    CHECK(e.srcV == -1);
    CHECK(e.weight == -1);

    // Insert and extract again to validate behavior
    h.insert(0, 1, 5);
    h.insert(1, 2, 3);
    h.insert(2, 3, 7);

    CHECK(!h.isEmpty());

    edge e1 = h.extractMin();
    CHECK(e1.weight == 3);

    e1 = h.extractMin();
    CHECK(e1.weight == 5);

    e1 = h.extractMin();
    CHECK(e1.weight == 7);
}

TEST_CASE("unionFind operations") {
    unionFind uf;

    // Initially each element is its own root
    CHECK(uf.find(1) == 1);

    // Perform union and verify connected components
    uf.unionSet(1, 2);
    CHECK(uf.find(1) == uf.find(2));
}