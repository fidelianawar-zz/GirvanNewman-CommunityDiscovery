//
// Created by Fidelia Nawar on 2/27/20.
//

#ifndef INC_20S_3353_PA02_GRAPH_H
#define INC_20S_3353_PA02_GRAPH_H

#include <vector>
#include <list>
#include <iostream>
#include <iterator>
#include <string>
#include <algorithm>

using std::vector;
using std::list;
using std::cout;
using std::endl;
using std::iterator;

template<class T>
struct Vertex{
    T source, destination;
};

template<class T>
class Graph {

private:
    int numVertices;
    list<list<T>> adjacencyList;
    typename std::list<T>::iterator it;

public:

    // Graph Constructor
    Graph(int V);


    void addEdge(T u, T v);
    void displayGraph(int v);

    void addVertex(list<list<T>> adjList, T u){
//        AdjListNode* newNode = new AdjListNode;
//        newNode->dest = dest;
//        newNode->next = NULL;
//        return newNode;
     }

     void printGraph(vector<int> adj[], int V)
    {
        for (int v = 0; v < V; ++v)
        {
            std::cout << "\n Adjacency list of vertex " << v << "\n head ";
            for (auto x : adj[v])
                std::cout << "-> " << x;
            printf("\n");
        }
    }
};

template<class T>
Graph<T>::Graph(int V) {
    this->numVertices = V;
}

template<class T>
void Graph<T>::addEdge(T u, T v){
    //auto it = std::next(adjacencyList.begin(), u);

    //iterate through adjacency list
    //check to see if vertex u exists
    //if it does, push back v
    //else, create new u
    for(int i = 0; i < numVertices; i++){
        if(it == u){
            adjacencyList.push_back(v);
        }
        else{

        }
    }

    it = adjacencyList.begin();
    std::advance(it, u);
    //adjacencyList.push_back(v);
    //adjacencyList[u].push_back(v);
}

template<class T>
// A utility function to print the adjacency list representation of graph
void Graph<T>::displayGraph(int v){
    for(int i = 0; i < v; i++){
        cout << i << "---->";
        //list<T> :: iterator it;
        for(auto it  = adjacencyList[i].begin(); it != adjacencyList[i].end(); ++it) {
            cout << *it << " ";
        }
        cout << endl;
    }
}

#endif //INC_20S_3353_PA02_GRAPH_H