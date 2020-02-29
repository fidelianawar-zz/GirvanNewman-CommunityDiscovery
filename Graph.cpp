//
// Created by Fidelia Nawar on 2/27/20.
//

#include "Graph.h"

template<class T>
Graph::Graph(int V){
    this->numVertices = V;
    adjacencyList = new list[numVertices];
    for (int i = 0; i < numVertices; ++i){
        adjacencyList[i].head = NULL;
    }
}

template<class T>
//add v into the list u
void Graph::addEdge(list<list<T>> adjList, T u, T v){
    adjacencyList[u].push_back(v);
}

template<class T>
void Graph::addVertex(list<list<T>> adjList, int v){

}

template<class T>
// A utility function to print the adjacency list representation of graph
void displayGraph(list<list<T> adjList, int v){
    for(int i = 0; i < v; i++){
        cout << i << "---->";
        list<T> :: iterator it;
        for(it = adjList[i].begin(); it != adjList[i].end(); ++it) {
            cout << *it << " ";
        }
        cout << endl;
    }
}
