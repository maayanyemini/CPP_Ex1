//

#ifndef QUEUE_H
#define QUEUE_H
#define MAX_SIZE 100


namespace graph {
    class Queue {
    private:
        int data[MAX_SIZE]{};
        int front, rear;

    public:
        Queue();

        void enqueue(int value);

        int dequeue();

        bool isEmpty() const;
    };

    struct edge {
        int srcV;
        int dstV;
        int weight;
    };

    class minHeap {
    private:
        edge data[MAX_SIZE]{};
        int size;

    public:
        minHeap();

        void insert(int src, int dst, int weight);

        edge extractMin();

        bool isEmpty() const;
    };

    class unionFind {
    private:
        int parent[MAX_SIZE]{};

    public:
        unionFind();

        int find(int v);

        void unionSet(int u, int v);
    };
}

#endif
