//


#ifndef GRAPH_H
#define GRAPH_H


namespace graph {
    struct Neighbor {
        int vertex;
        int weight;
        Neighbor *next;
    };

    class Graph {
    private:
        const int numV;
        Neighbor **adjacencyList;

    public:
        explicit Graph(int vertices);
        
        Graph(const Graph &g);

        ~Graph();

        int get_numV() const;

        Neighbor *getNeighbors(int vertex) const;

        void addNeighbor(int src, int dst, int weight = 1);

        void addEdge(int src, int dst, int weight = 1);

        void operator=(const Graph &g);

        bool edgeExists(int src, int dst) const;

        void removeEdge(int src, int dst);

        void removeEdgeFromTo(int src, int dst);

        void print_graph() const;

    };
}

#endif
