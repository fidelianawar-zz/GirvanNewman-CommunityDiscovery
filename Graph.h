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
    std::unordered_map<T, int> reverseVertexMap;
    vector<vector<T>> fullPath;
    vector<T> tempPath;
    int *storedPath;
    int count = 0;
    typename std::unordered_map<T, int>:: iterator itr;

public:

    Graph();
    void populateAdj(T src, T dest);
    void displayAdjVec();
    void hashVertex(T);
    void createAdj(int);

    void makeConnection(T, T);
    void printMaps();
    void getAllPaths(T s, T d);
    void getAllPathsHelper(int u, int d, bool visited[],
                      int path[], int &path_index);

    void DFS(T);
    void DFSHelper(int v, bool visited[]);
    void BFS(T);
    void girvanNewmanAlgo(T);
    void removeEdge(T,T); //remove edges based on betweenness value

};

template<class T>
Graph<T>::Graph() {
    cout << "making a graph";
}

template<class T>
void Graph<T>::hashVertex(T vertex) {
    //A - 0, B - 1, C -2, etc.
    vertexMap.insert(std::make_pair(vertex, count));
//    reverseVertexMap.insert(std::make_pair(count,vertex));
    //cout << reverseVertexMap.size();
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
void Graph<T>::displayAdjVec(){
    cout << endl << "Adjacency List" << endl;
    for(unsigned int i  = 0; i < adjVec.size(); i++){
        cout << i << "-> ";
        for(unsigned int j = 0; j < adjVec[i].size(); j++){
            cout << adjVec[i][j] << " ";
        }
        cout << endl;
    }
    cout << endl;
}

template<class T>
void Graph<T>::DFS(T node) {
    cout << "inside Graph.h dfs" << endl;
    int value = vertexMap.at(node);
    cout << "Node key is: " << value << endl << endl;

    // Mark all the vertices as not visited
    bool *visited = new bool[adjVec.size()];
    for (unsigned int i = 0; i < adjVec.size(); i++){
       visited[i] = false;
    }

    // Call the recursive helper function to print DFS traversal
    cout << "DFS traversal of " << node << " is: ";
    DFSHelper(value, visited);
}

template<class T>
void Graph<T>::DFSHelper(int v, bool visited[])
{
    visited[v] = true;
    cout << v << " ";
    // Recur for all the vertices adjacent to this vertex
    vector<int>::iterator i;
    for(i = adjVec[v].begin(); i != adjVec[v].end(); i++){
        if(!visited[*i]){
            DFSHelper(*i, visited);
        }
    }
}

template<class T>
void Graph<T>::BFS(T node){
    int value = vertexMap.at(node);
    bool *visited = new bool[adjVec.size()];
    for(unsigned int i = 0; i < adjVec.size(); i++){
        visited[i] = false;
    }

    list<int> queue;
    visited[value] = true;
    queue.push_back(value);

    vector<int>::iterator i;

    cout << endl << "BFS traversal of " << node << " is: ";
    while(!queue.empty()){
        value = queue.front();
        cout << value << " ";
        queue.pop_front();

        for(i = adjVec[value].begin(); i != adjVec[value].end(); i++){
            if(!visited[*i]){
                visited[*i] = true;
                queue.push_back(*i);
            }
        }
    }
    cout << endl << endl;
}

template<class T>
void Graph<T>::girvanNewmanAlgo(T node) {
    BFS(node);
}

template<class T>
// Prints all paths from 's' to 'd'
void Graph<T>::getAllPaths(T source, T destination)
{
    int start = vertexMap.at(source);
    int dest = vertexMap.at(destination);

    // Mark all the vertices as not visited
    bool *visited = new bool[adjVec.size()];

    // Create an array to store paths
    int *path = new int[adjVec.size()];
    int path_index = 0; // Initialize path[] as empty

    // Initialize all vertices as not visited
    for (unsigned int i = 0; i < adjVec.size(); i++)
        visited[i] = false;

    // Call the recursive helper function to print all paths
    cout << endl << source << "(" << start << ") -> " << destination << "(" << dest << ") paths are: " << endl;
    getAllPathsHelper(start, dest, visited, path, path_index);

}

template<class T>
void Graph<T>::getAllPathsHelper(int u, int d, bool visited[], int path[], int &path_index)
{
    // Mark the current node and store it in path[]
    visited[u] = true;
    path[path_index] = u;
    path_index++;

    // If current vertex is same as destination, then print current path[]
    if (u == d)
    {
        for (int i = 0; i<path_index; i++){
            cout << path[i] << " ";
        }
    }

    else // If current vertex is not destination
    {
        // Recur for all the vertices adjacent to current vertex
        vector<int>::iterator i;
        for (i = adjVec[u].begin(); i != adjVec[u].end(); ++i)
            if (!visited[*i])
                getAllPathsHelper(*i, d, visited, path, path_index);
    }

    // Remove current vertex from path[] and mark it as unvisited
    path_index--;
    visited[u] = false;
}

template <class T>
void Graph<T>::printMaps() {
    for (auto itr = vertexMap.begin(); itr != vertexMap.end(); ++itr) {
        cout << itr->first << "\t" << itr->second << "\t";
    }
}
template <class T>
void Graph<T>::makeConnection(T s, T d) {
    cout << "inside of makeConnection";
    int connector = vertexMap.at(s);
    int connecting = vertexMap.at(d);



}

#endif //INC_20S_3353_PA02_GRAPH_H