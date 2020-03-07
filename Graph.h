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
    vector<vector<int>> adjVec;
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
    void populateAdj(T src, T dest);
    void displayGraph();
    void displayAdjVec();
    void testFunction();
    void hashVertex(T);
    void createAdj(int);


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
    vertexMap.insert(std::make_pair(v, count));
    count++;
}

template<class T>
void Graph<T>::createAdj(int numV){
    cout << "number of vertices is: " << numV << endl;
    vector<int> emptyVector;
    for(int i = 0; i < numV; i++){
        adjVec.push_back(emptyVector);
    }
    cout << endl;
}

template<class T>
void Graph<T>::populateAdj(T src, T dest) { //src: vertex, dest: edge to be added
    int srcKey = vertexMap.at(src); //hash index for src
    int destKey = vertexMap.at(dest); //hash index for dest
    cout << "src key : " << srcKey << " " << "dest key: " << destKey << " " << endl;
    //cout << srcKey << " " <<  destKey << endl;
    (adjVec.at(srcKey)).push_back(destKey);
}

template<class T>
// A utility function to print the adjacency list representation of graph
void Graph<T>::displayAdjVec() {
    for (unsigned int i = 0; i < adjVec.size(); i++) {
        vector<int> tempVec = adjVec[i];
        cout << "HEAD:  " << i << " ---> ";
        for (int j = tempVec[0]; j < tempVec[tempVec.size()-1]; j++) {
            cout << j << " ";
        }
        cout << endl;
    }
}

template<class T>
void Graph<T>::testFunction(){
    for(int i  = 0; i < adjVec.size(); i++){
        cout << i << "-> ";
        for(int j = 0; j < adjVec[i].size(); j++){
            cout << adjVec[i][j] << " ";
        }
        cout << endl;
    }
}

template<class T>
vector<T> Graph<T>::performDFS(std::basic_string<char> node) {
   cout << "made it here";
}


#endif //INC_20S_3353_PA02_GRAPH_H