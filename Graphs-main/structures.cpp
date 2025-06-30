#include "structures.h"
#include <iostream>
using namespace std;

namespace graph {

    // Initialize queue indices (circular buffer setup)
    Queue::Queue() : front(0), rear(0) {}

    // Insert element at rear of the queue if space allows
    void Queue::enqueue(int value) {
        if ((rear + 1) % MAX_SIZE == front) {
            cout << "Queue overflow: Cannot enqueue." << endl;
        } else {
            data[rear] = value;
            rear = (rear + 1) % MAX_SIZE;
        }
    }

    // Remove and return front value, or -1 if queue is empty
    int Queue::dequeue() {
        if (front == rear) {
            cout << "Queue underflow: Nothing to dequeue." << endl;
            return -1;
        }
        int val = data[front];
        front = (front + 1) % MAX_SIZE;
        return val;
    }

    // Determine whether the queue currently holds elements
    bool Queue::isEmpty() const {
        return front == rear;
    }

    // Set up an empty minHeap
    minHeap::minHeap() : size(0) {}

    // Insert a new edge and maintain descending weight order
    void minHeap::insert(int src, int dst, int weight) {
        if (size == MAX_SIZE) {
            cout << "Heap is full: Cannot insert edge." << endl;
            return;
        }
        data[size] = {src, dst, weight};
        int idx = size;
        size++;

        // Push the edge backward until order is maintained
        while (idx > 0 && data[idx].weight > data[idx - 1].weight) {
            swap(data[idx], data[idx - 1]);
            idx--;
        }
    }

    // Return edge with minimum weight (last element in descending heap)
    edge minHeap::extractMin() {
        if (size == 0) {
            cout << "Heap is empty: Cannot extract." << endl;
            return {-1, -1, -1};
        }
        return data[--size];
    }

    // Check if the heap has no elements
    bool minHeap::isEmpty() const {
        return size == 0;
    }

    // Create disjoint sets where each element is its own leader
    unionFind::unionFind() {
        for (int i = 0; i < MAX_SIZE; ++i) {
            parent[i] = i;
        }
    }

    // Retrieve representative/root of the set containing vertex v
    int unionFind::find(int v) {
        while (parent[v] != v) {
            v = parent[v];
        }
        return v;
    }

    // Merge two disjoint sets containing u and v respectively
    void unionFind::unionSet(int u, int v) {
        int rootU = find(u);
        int rootV = find(v);
        if (rootU != rootV) {
            parent[rootU] = rootV;
        }
    }

} // namespace graph
