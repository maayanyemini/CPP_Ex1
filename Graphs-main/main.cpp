

#include <iostream>
#include "Graph.h"
#include "Algorithms.h"

using namespace graph;
using namespace std;


int main() {
    Graph g(6);

    g.addEdge(0,1,4);
    g.addEdge(0,2,3);
    g.addEdge(1,3,2);
    g.addEdge(1,4,7);
    g.addEdge(2,4,1);
    g.addEdge(3,5,5);
    g.addEdge(4,5,2);

    cout << "Original Graph:" << endl;
    g.print_graph();



    Graph bfsTree= Algorithms::bfs(g,0);
    cout << "after bfs" << endl;
    bfsTree.print_graph();


    Graph dfsTree= Algorithms::dfs(g,0);
    cout << "after dfs" << endl;
    dfsTree.print_graph();


    Graph dTree=Algorithms::dijkstra(g,0);
    cout << "after dijkstra:" << endl;
    dTree.print_graph();


    Graph prim=Algorithms::prim(g);
    cout << "after prim:" << endl;
    prim.print_graph();


    Graph kruskalTree=Algorithms::kruskal(g);
    cout << "after kruskal:" << endl;
    kruskalTree.print_graph();



    return 0;


}

// TIP See CLion help at <a
// href="https://www.jetbrains.com/help/clion/">jetbrains.com/help/clion/</a>.
//  Also, you can try interactive lessons for CLion by selecting
//  'Help | Learn IDE Features' from the main menu.