//
// Created by Fidelia Nawar on 2/27/20.
//
#ifndef INC_20S_3353_PA02_GRAPH_H
#define INC_20S_3353_PA02_GRAPH_H

#include <iostream>
#include <vector>
#include <list>
#include <map>
#include <unordered_map>

using std::cout;
using std::cin;
using std::endl;
using std::string;
using std::vector;
using std::list;

template<class T>
class Graph {
    int numVertices;
    vector<list<T>> adjLists;
    vector<vector<T>> adjVec;
    vector<vector<T>> testVec;
    list<T> innerList;
    vector<T> tempVec;
    std::unordered_map<T, int> vertexMap;
    int count = 0;
    typename std::unordered_map<T, int>:: iterator itr;

public:

    Graph(int V);
    void addVertex(T &);
    void addVertexVec(T);
    void addEdge(T& src, T& dest);
    void addEdgeVec(T src, T dest);
    void displayGraph();
    void displayGraphVec();
    void testFunction();
    void hashVertex(T);


    vector<T> performDFS(std::basic_string<char>);

    int getSize();
};

template<class T>
Graph<T>::Graph(int vertices) {
    this->numVertices = vertices;
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

//////////////////////////////////////////////////////////////////////////////////////////

template<class T>
void Graph<T>::hashVertex(T v) {
    cout << "The vertex is: " << v << endl;
    vertexMap.insert(std::make_pair(v, count));
    count++;
    for (itr = vertexMap.begin(); itr != vertexMap.end(); itr++)
    {
        cout << itr->first << "  " << itr->second << endl;
    }
}
template<class T>
void Graph<T>::addVertexVec(T v) {
    cout << "vertex is: " << v << endl; //this outputs the correct vertex
    vector<T> tempVec;
    tempVec.push_back(v);
    cout << tempVec.size();
    T element = tempVec[0];
    cout << "first element of tempVec is: " << tempVec[0] << endl;
    testVec.push_back(tempVec);
    //cout << adjVec[0];
}

template<class T>
void Graph<T>::addEdgeVec(T src, T dest) { //src: vertex, dest: edge to be added
    cout << "src: " << src << " " << "dest: " << dest << endl;
    //for()

}
template<class T>
// A utility function to print the adjacency list representation of graph
void Graph<T>::displayGraphVec() {
    for (unsigned int i = 0; i < testVec.size(); i++) {
        vector<T> tempVec = testVec[i];
        auto val = tempVec[0];
        cout << "HEAD:  " << val << "--->";
        for (auto it2 = 1; it2 != tempVec.size(); it2++) {
            cout << it2 << " ";
        }
        cout << endl;
    }
}

template<class T>
void Graph<T>::testFunction(){
    cout << testVec.size() << endl;
    cout << "the first element of Adj Vec is of size: " << testVec[0].size() << endl;
    cout << "the first element of the first vec is: " << testVec[0][0] << endl;
}

template<class T>
vector<T> Graph<T>::performDFS(std::basic_string<char> node) {
   cout << "made it here";
}


#endif //INC_20S_3353_PA02_GRAPH_H