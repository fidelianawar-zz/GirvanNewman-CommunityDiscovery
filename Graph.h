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
    cout << "vertex is: " << &v << endl; //this outputs the correct vertex
    list<T> tempList;
    tempList.emplace_back(v);
    cout << tempList.size();
    auto ele = tempList.front();
    cout << " the front of tempList is: " << ele << endl;
    adjLists.push_back(tempList); //adjList is of type vector<list<T>>
}

template<class T>
void Graph<T>::addEdge(T& src, T& dest) { //src: vertex, dest: edge to be added
    cout << "src: " << &src << " " << "dest: " << &dest << endl;
    list<T> tempList;
    for (unsigned int i = 0; i < adjLists.size(); i++) {
        //create tempList for each sublist in adjList
        tempList = adjLists[i];
        //grab first element of tempList
        auto val = next(tempList.begin(), 0);
        cout << "front of list[" << i << "]: " << tempList.front() << endl;
        //check if first element == 'src' parameter
        if (*val == src) {
            tempList.push_back(dest);
        }
    }
}
template<class T>
int Graph<T>::getSize() {
    return adjLists.size();
}
template<class T>
// A utility function to print the adjacency list representation of graph
void Graph<T>::displayGraph() {
    for (unsigned int i = 0; i < adjLists.size(); i++) {
        list<T> tempList = adjLists[i];
        auto val = next(tempList.begin(), 0);
        cout << "HEAD:  " << *val << "--->";
        for (auto it2 = next(tempList.begin(), 1); it2 != tempList.end(); it2++) {
            cout << *it2 << " ";
        }
        cout << endl;
    }
}


#endif //INC_20S_3353_PA02_GRAPH_H