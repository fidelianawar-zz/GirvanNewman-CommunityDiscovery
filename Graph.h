//
// Created by Fidelia Nawar on 2/27/20.
//

#ifndef INC_20S_3353_PA02_GRAPH_H
#define INC_20S_3353_PA02_GRAPH_H

#include <vector>
#include <list>
#include <iostream>

using std::vector;
using std::list;

template<class T>
class Graph {

    int numVertices;
    list<list<T>> adjacencyList;

    // Graph Constructor
    Graph(int);
    void addEdge(list<list<T>>, T, T);

    void addVertex(list<list<T>> adjList, T u){
        AdjListNode* newNode = new AdjListNode;
        newNode->dest = dest;
        newNode->next = NULL;
        return newNode;
    }

    void displayAdjList(list<list<T> adjList, int v);

    void printGraph(vector<int> adj[], int V)
    {
        for (int v = 0; v < V; ++v)
        {
            cout << "\n Adjacency list of vertex " << v << "\n head ";
            for (auto x : adj[v])
                cout << "-> " << x;
            printf("\n");
        }
    }
};


#endif //INC_20S_3353_PA02_GRAPH_H
