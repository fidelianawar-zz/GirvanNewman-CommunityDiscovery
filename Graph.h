//
// Created by Fidelia Nawar on 2/27/20.
//
#ifndef INC_20S_3353_PA02_GRAPH_H
#define INC_20S_3353_PA02_GRAPH_H

#include <iostream>
#include <vector>
#include <list>
#include <map>

using namespace std;

template<class T>
class Graph {
    int numVertices;
    vector<list<T>> adjLists;
    list<T> innerList;

public:
    Graph(int V);
    void addVertex(T &);
    void addEdge(T& src, T& dest);
    void displayGraph();
    int getSize();
};

template<class T>
Graph<T>::Graph(int vertices) {
    numVertices = vertices;
}

template<class T>
void Graph<T>::addVertex(T &v) {
    cout << "vertex is: " << &v << endl;
    list<T> tempList;
    innerList.push_back(v);
    adjLists.push_back(innerList);
}

template<class T>
void Graph<T>::addEdge(T& src, T& dest) { //src: vertex, dest: edge to be added
    list<T> tempList;
    for (int i = 0; i < adjLists.size(); i++) {
        //create tempList for each sublist in adjList
        tempList = adjLists[i];
        //grab first element of tempList
        auto val = next(tempList.begin(), 0);
        //check if first element == 'src' parameter
        if (*val == src) {
            //if 'dest' doesnt exist, BUT vertex does, push to tempList;
            innerList.push_back(dest);
        }
    }
    cout << "size of adjList is: " << adjLists.size() << endl;
}

template<class T>
int Graph<T>::getSize() {
    return adjLists.size();
}

template<class T>
// A utility function to print the adjacency list representation of graph
void Graph<T>::displayGraph() {
    for (int i = 0; i < adjLists.size(); i++) {
        list<T> tempList = adjLists[i];
        for (auto it2 = tempList.begin(); it2 != tempList.end(); it2++) {
            cout << " elements of  tempList: " << *it2 << " ";
        }
        cout << endl;
    }
}


#endif //INC_20S_3353_PA02_GRAPH_H