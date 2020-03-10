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
#include <limits.h>
#include <climits>
#include <cstdint>

using std::cout;
using std::cin;
using std::endl;
using std::string;
using std::vector;
using std::list;

template<class T>
class InfoTracker{
    bool visited;
    int distanceFromOrigin;
    T parent;
};

template<class T>
class Graph {

    vector<vector<int>> adjVec;
    std::unordered_map<T, int> vertexMap;
    std::unordered_map<int, T> reverseVertexMap;
    int count = 0;

    vector<int> bfsVector;
    vector<int> dfsVector;
    vector<int> edgePath;
    vector<int> shortestPathVector;

    vector<std::pair<int, int>> bfsEdgeList;
    vector<std::pair<int, int>> dfsEdgeList;
    vector<std::pair<int,int>> shortestPathEdgeList;

public:

    Graph();
    T unHash(int);
    void createReverseVertexMap();
    void populateAdj(T src, T dest);
    void displayAdjVec();
    void hashVertex(T);
    void createAdj(int);

    void findNumEdges(T, T);
    void makeConnection(T,T);
    void printShortestDistance(int s, int dest);

    void printMaps();
    T getKey(T value);
    void getAllPaths(T s, T d);
    void getAllPathsHelper(int u, int d, bool visited[],
                      int path[], int &path_index);

    void DFS(T);
    void DFSHelper(int v, bool visited[]);
    void BFS(T);
    bool connectionBFS(int src, int dest, int pred[], int dist[]);

    bool isConnected(int src, int dest, vector<bool> &discovered, vector<int> &path);
    void createShortestPathEdgeList();
    void createBFSEdgeList();
    void createDFSEdgeList();
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
    count++;
}

template <class T>
void Graph<T>::createReverseVertexMap() {
    //0 - A, 1 - B, 2 - C, etc.
    for (auto itr = vertexMap.begin(); itr != vertexMap.end(); ++itr) {
        reverseVertexMap[itr->second] = itr->first;
    }
}

template<class T>
T Graph<T>::unHash(int vertex) {
    return reverseVertexMap[vertex];
}

template<class T>
void Graph<T>::createAdj(int numV){
    cout << "number of vertices is: " << numV << endl;
    vector<int> emptyVector;
    for(int i = 0; i < numV; i++){
        adjVec.push_back(emptyVector);
    }
    createReverseVertexMap();
    cout << endl;
}

template<class T>
void Graph<T>::populateAdj(T src, T dest) { //src: vertex, dest: edge to be added
    int srcKey = vertexMap.at(src); //hash index for src
    int destKey = vertexMap.at(dest); //hash index for dest
    cout << "src key : " << unHash(srcKey) << " " << "dest key: " << unHash(destKey) << " " << endl;
    (adjVec.at(srcKey)).push_back(destKey);
    (adjVec.at(destKey)).push_back(srcKey);
}

template<class T>
void Graph<T>::displayAdjVec(){
    cout << endl << "Adjacency List" << endl;
    for(unsigned int i  = 0; i < adjVec.size(); i++){
        cout << unHash(i) << "-> ";
        for(unsigned int j = 0; j < adjVec[i].size(); j++){
            cout << unHash(adjVec[i][j]) << " ";
        }
        cout << endl;
    }
    cout << endl;
}

template<class T>
void Graph<T>::DFS(T node) {

    int value = vertexMap.at(node);
    cout << endl << "Node is: " << unHash(value) << endl;

    // Mark all the vertices as not visited
    bool *visited = new bool[adjVec.size()];
    for (unsigned int i = 0; i < adjVec.size(); i++){
       visited[i] = false;
    }

    // Call the recursive helper function to print DFS traversal
    cout << "DFS traversal of " << node << " is: ";
    DFSHelper(value, visited);
    createDFSEdgeList();
}

template<class T>
void Graph<T>::DFSHelper(int v, bool visited[])
{
    visited[v] = true;
    cout << unHash(v) << " ";
    dfsVector.push_back(v);
    // Recur for all the vertices adjacent to this vertex
    vector<int>::iterator i;
    for(i = adjVec[v].begin(); i != adjVec[v].end(); i++){
        if(!visited[*i]){
            DFSHelper(*i, visited);
        }
    }
}
template<class T>
void Graph<T>::createDFSEdgeList() {
    dfsEdgeList.push_back(std::make_pair(dfsVector[0], dfsVector[1]));
    for (unsigned int i = 1; i < dfsVector.size(); i++) {
        dfsEdgeList.push_back(std::make_pair(dfsVector[i], dfsVector[i + 1]));
    }
    cout << endl << "DFS Edge List: (";
    for (auto itr = dfsEdgeList.begin(); itr != dfsEdgeList.end()-1; ++itr) {
        if(itr == dfsEdgeList.end()-2){
            cout << "{" << unHash(itr->first) << " - " << unHash(itr->second) << "}";
        }
        else{
            cout << "{" << unHash(itr->first) << " - " << unHash(itr->second) << "} ";
        }
    }
    cout << ")" << endl;
}
template<class T>
void Graph<T>::BFS(T node) {
    int value = vertexMap.at(node);
    cout << endl << "Node is: " << unHash(value) << endl;
    bool *visited = new bool[adjVec.size()];
    for (unsigned int i = 0; i < adjVec.size(); i++) {
        visited[i] = false;
    }

    list<int> queue;
    visited[value] = true;
    queue.push_back(value);

    vector<int>::iterator i;

    cout << "BFS traversal of " << node << " is: ";
    while (!queue.empty()) {
        value = queue.front();
        bfsVector.push_back(value);
        cout << unHash(value) << " ";
        queue.pop_front();

        for (i = adjVec[value].begin(); i != adjVec[value].end(); i++) {
            if (!visited[*i]) {
                visited[*i] = true;
                queue.push_back(*i);
            }
        }
    }
    cout << endl;
    createBFSEdgeList();
}

template<class T>
void Graph<T>::createBFSEdgeList() {
    bfsEdgeList.push_back(std::make_pair(bfsVector[0], bfsVector[1]));
    for (unsigned int i = 1; i < bfsVector.size(); i++) {
        bfsEdgeList.push_back(std::make_pair(bfsVector[i], bfsVector[i + 1]));
    }
    cout << "BFS Edge List: (";
    for (auto itr = bfsEdgeList.begin(); itr != bfsEdgeList.end()-1; ++itr) {
        if(itr == bfsEdgeList.end()-2){
            cout << "{" << unHash(itr->first) << " - " << unHash(itr->second) << "}";
        }
        else {
            cout << "{" << unHash(itr->first) << " - " << unHash(itr->second) << "} ";
        }
    }
    cout << ")" << endl;
}

template<class T>
void Graph<T>::girvanNewmanAlgo(T node) {
    BFS(node);
}

template<class T>
T Graph<T>::getKey(T value) {
    typename std::unordered_map<int, T>::iterator it;
    T key;
    for (it = vertexMap.begin(); it != vertexMap.end(); ++it){
        if (it->second == value){
            key = it->first;
            break;
        }
    }
    return key;
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
    if (u == d){
        for (int i = 0; i<path_index; i++){
            cout << path[i] << " ";
        }
    }

    else{ // If curr vertex is not destination, recur for all the vertices adjacent to current vertex
        vector<int>::iterator i;
        for (i = adjVec[u].begin(); i != adjVec[u].end(); ++i){
            if (!visited[*i]){
                getAllPathsHelper(*i, d, visited, path, path_index);
            }
        }
    }

    // Remove current vertex from path[] and mark it as unvisited
    path_index--;
    visited[u] = false;
}

template <class T>
void Graph<T>::printMaps() {
    for (auto itr = reverseVertexMap.begin(); itr != reverseVertexMap.end(); ++itr) {
        cout << itr->first << "\t" << itr->second << "\t" << endl;
    }
}

template<class T>
void Graph<T>::findNumEdges(T ctr, T cting) {
    // visited[n] for keeping track of visited
    // node in BFS
    int connector = vertexMap.at(ctr);
    int connecting = vertexMap.at(cting);

    vector<bool> visited(adjVec.size(), 0);

    // Initialize distances as 0
    vector<int> distance(adjVec.size(), 0);

    // queue to do BFS.
    std::queue <int> Q;
    distance[connector] = 0;

    Q.push(connector);
    visited[connector] = true;
    while (!Q.empty()){
        int x = Q.front();
        Q.pop();

        for (int i=0; i<adjVec[x].size(); i++){
            if (visited[adjVec[x][i]]){
                continue;
            }
            // update distance for i
            distance[adjVec[x][i]] = distance[x] + 1;
            Q.push(adjVec[x][i]);
            visited[adjVec[x][i]] = 1;
        }
    }
    cout << endl << "Number of edges between " << unHash(connector) << " and " << unHash(connecting) << " is: " <<
    distance[connecting] << endl;
    vector<bool> discovered(adjVec.size(), 0);
    isConnected(connector, connecting, discovered,edgePath);
    cout << "The edge path from " << unHash(connector) << " to " << unHash(connecting) << " is: ";
    for (int i: edgePath)
        cout << unHash(i) << ' ';
    cout << endl;
}

// Function to perform DFS traversal in a directed graph to find the
// complete path between source and destination vertices
template<class T>
bool Graph<T>::isConnected(int src, int dest, vector<bool> &discovered, vector<int> &path){
    // mark current node as discovered
    discovered[src] = true;

    // include current node in the path
    path.push_back(src);

    // if destination vertex is found
    if (src == dest) {
        return true;
    }
    // do for every edge (src -> i)
    for (int i: adjVec[src]){
        // u is not discovered
        if (!discovered[i]){
            // return true if destination is found
            if (isConnected(i, dest, discovered, path))
                return true;
        }
    }
    // backtrack: remove current node from the path
    path.pop_back();

    // return false if destination vertex is not reachable from src
    return false;
}

/*
 * Make a Connection with the smallest number of introductions.
 * A set of introductions that need to be made in order for person A
 * to be introduced to person D.
 * Ex: {(A - B), (B - D)}
 */
template<class T>
void Graph<T>::makeConnection(T s, T d) {
    int src = vertexMap.at(s);
    int dest = vertexMap.at(d);
    printShortestDistance(src,dest);
}

template<class T>
bool Graph<T>::connectionBFS(int src, int dest, int pred[], int dist[])
{
    //queue for BFS
    list<int> queue;

    //checks if nodes are visited
    bool visited[adjVec.size()];

    //setting initial values to false/infinity
    for (unsigned int i = 0; i < adjVec.size(); i++) {
        visited[i] = false;
        dist[i] = INT_MAX;
        pred[i] = -1;
    }

    //visit source and set distance to 0
    visited[src] = true;
    dist[src] = 0;
    queue.push_back(src);

    //BFS 2.0
    while (!queue.empty()) {
        int u = queue.front();
        queue.pop_front();
        for (unsigned int i = 0; i < adjVec[u].size(); i++) {
            if (visited[adjVec[u][i]] == false) {
                visited[adjVec[u][i]] = true;
                dist[adjVec[u][i]] = dist[u] + 1;
                pred[adjVec[u][i]] = u;
                queue.push_back(adjVec[u][i]);

                //stop when destination is found
                if (adjVec[u][i] == dest)
                    return true;
            }
        }
    }

    return false;
}

template <class T>
void Graph<T>::printShortestDistance(int s, int dest)
{
    // predecessor[i] contains parent of i and dist[] stores distance of i from source
    int pred[adjVec.size()], dist[adjVec.size()];

    if (connectionBFS(s, dest, pred, dist) != true)
    {
        cout << "Given source and destination cannot be connected";
        return;
    }

    int crawl = dest;
    shortestPathVector.push_back(crawl);
    while (pred[crawl] != -1) {
        shortestPathVector.push_back(pred[crawl]);
        crawl = pred[crawl];
    }

    // distance from source is in distance array
    cout << endl << "Shortest path length is : " << dist[dest];

    // printing path from source to destination
    cout << "\nTo travel from " << unHash(s) << " to " << unHash(dest) << " , the path is: " << endl;
    for (int i = shortestPathVector.size() - 1; i >= 0; i--){
        cout << unHash(shortestPathVector[i]) << " ";
    }
    cout << endl << endl;
    createShortestPathEdgeList();
}

template <class T>
void Graph<T>::createShortestPathEdgeList() {

    std::reverse(shortestPathVector.begin(), shortestPathVector.end());
    shortestPathEdgeList.push_back(std::make_pair(shortestPathVector[0], shortestPathVector[1]));

    for (unsigned int i = 1; i < shortestPathVector.size(); i++) {
        shortestPathEdgeList.push_back(std::make_pair(shortestPathVector[i], shortestPathVector[i + 1]));
    }
    cout << "Shortest Path Edge List: (";
    for (auto itr = shortestPathEdgeList.begin(); itr != shortestPathEdgeList.end()-1; ++itr) {
        if(itr == shortestPathEdgeList.end()-2){
            cout << "{" << unHash(itr->first) << " - " << unHash(itr->second) << "}";
        }
        else {
            cout << "{" << unHash(itr->first) << " - " << unHash(itr->second) << "} ";
        }
    }
    cout << ")" << endl;
}

#endif //INC_20S_3353_PA02_GRAPH_H