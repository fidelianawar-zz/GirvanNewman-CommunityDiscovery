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
#include <algorithm>
#include <string>
#include <queue>

using std::cout;
using std::cin;
using std::endl;
using std::string;
using std::vector;
using std::list;

template<class T>
class Graph {

    vector<vector<int>> adjVec;
    std::unordered_map<T, int> vertexMap;
    std::unordered_map<int, T> reverseVertexMap;
    int count = 0;
    typename std::unordered_map<T, int>:: iterator itr;

public:

    Graph();
    void addVertex(T &);
    void addVertexVec(T);
    void addEdge(T& src, T& dest);
    void populateAdj(T src, T dest);
    void displayGraph();
    void displayAdjVec();
    void testFunction();
    void hashVertex(T);
    void createAdj(int);


    void dfs(T);
    void bfs(T);
    void dfsHelper(int v, bool visited[]);

    int getSize();
};

template<class T>
Graph<T>::Graph() {
    cout << "making a graph bitch";
}

//////////////////////////////////////////////////////////////////////////////////////////

template<class T>
void Graph<T>::hashVertex(T vertex) {
    //A - 0, B - 1, C -2, etc.
    vertexMap.insert(std::make_pair(vertex, count));
    reverseVertexMap.insert(std::make_pair(count,vertex));
    cout << reverseVertexMap.size();
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
    (adjVec.at(destKey)).push_back(srcKey);
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
    for(unsigned int i  = 0; i < adjVec.size(); i++){
        cout << i << "-> ";
        for(unsigned int j = 0; j < adjVec[i].size(); j++){
            cout << adjVec[i][j] << " ";
        }
        cout << endl;
    }
}

template<class T>
void Graph<T>::dfs(T node) {
    cout << "inside Graph.h dfs" << endl;
    int value = vertexMap.at(node);
    cout << "Node key is: " << value << endl << endl;

    // Mark all the vertices as not visited
    bool *visited = new bool[adjVec.size()];
    for (int i = 0; i < adjVec.size(); i++){
       visited[i] = false;
    }

    // Call the recursive helper function to print DFS traversal
    dfsHelper(value, visited);
}

template<class T>
void Graph<T>::dfsHelper(int v, bool visited[])
{
    visited[v] = true;
    cout << "v is: " << v << " " << endl;

    // Recur for all the vertices adjacent to this vertex
    vector<int>::iterator i;
    int j;
    for(i = adjVec[v].begin(); i != adjVec[v].end(); i++){
//        for(j = adjVec[v][*i]; j < adjVec[v][adjVec[v].size()]; j++){
//            cout << "here";
//        }
    }
}

template<class T>
void Graph<T>::bfs(T node){
    std::queue<T> q;
}

#endif //INC_20S_3353_PA02_GRAPH_H