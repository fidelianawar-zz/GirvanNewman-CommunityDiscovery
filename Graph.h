//
// Created by Fidelia Nawar on 2/27/20.
//
#ifndef INC_20S_3353_PA02_GRAPH_H
#define INC_20S_3353_PA02_GRAPH_H

#include <iostream>
#include <fstream>
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
using std::ofstream;


template<class T>
class Graph {

    int numVertices;

    vector<vector<int>> adjVec;
    vector<int> numPathsSize;
    vector<vector<int>> allNumShortestPaths;
    vector<vector<int>> allPathsBetweenNodes;
    std::unordered_map<T, int> vertexMap;
    std::unordered_map<int, T> reverseVertexMap;
    std::map<std::pair<int,int>,int> betweennessMap;
    std::multimap<int,std::pair<int,int>> multimap;
    std::map<vector<int>, vector<int>> bigBoiMap;

    int count = 0;

    vector<int> dfsVector;
    vector<int> edgePath;
    vector<int> GirvanShortestPathVector;
    std::unordered_map<int, int> GparentChildrenCount;
    std::unordered_map<int, vector<int>> parentsMap;

    vector<std::pair<int, int>> dfsEdgeList;
    vector<std::pair<int, int>> shortestPathEdgeList;
    vector<std::pair<int, int>> GirvanShortestPathEdgeList;

public:

    Graph();

    T unHash(int);

    void createReverseVertexMap();

    void populateAdj(T src, T dest);

    void displayAdjVec();

    void hashVertex(T);

    void createAdj(int);

    void findNumEdges(T, T);

    void makeConnection(T, T, ofstream&);

    void printShortestDistance(int s, int dest, ofstream&);
    void findShortestDistancesGirvan(int s, int dest);

    void printMaps();

    T getKey(T value);

    int getAllPaths(T s, T d);

    int getAllPathsHelper(int u, int d, bool *visited, int *path, int &path_index);

    void printAllPaths();

    void DFS(T, ofstream& outputFile);

    void DFSHelper(int v, bool visited[], ofstream& outputFile);

    void BFS(T, ofstream& outputFile);

    vector<int> BFSPostEdgeDeletion(T);

    bool connectionBFS(int src, int dest, int pred[], int dist[]);

    bool isConnected(int src, int dest, vector<bool> &discovered, vector<int> &path);

    void createEdgeList(vector<int>, ofstream&);

    void createGirvanShortestPathEdgeList();

    void createBFSEdgeList(vector<int>, ofstream& outputFile);

    void createDFSEdgeList(ofstream& outputFile);

    void girvanNewman1();

    void removeNode(int); //remove edges based on betweenness value

    int isNotVisited(int x, vector<int> &path);

    int findpaths(vector<vector<int> > &g, int src,
                  int dst);

    void printPath(vector<int> &);

    void girvanNewman(ofstream&);

    void girvanNewman2();

    //void ModifiedBFS(T, T);

    bool BFSforGirvanAlgo(int src, int dest, int *parent, int *distanceFromParent);
    void printShortestDistanceG(int s, int dest);

    void girvanDFS(int,int);

    void girvan();

    void calculateBetweenness(vector<std::pair<int, int>>);
    void calculateBetweenness(int[]);
    void displayBetweennessMap();
    void removeEdges();
    void showCommunities(ofstream&);
    void displayBigBoyMap();
    void writeOutput(std::ofstream& output);

};

bool isNotVisited(int &i, vector<int> vector);

void printpath(vector<int> vector);

template<class T>
Graph<T>::Graph() {
    cout << "Making new graph.";
}

template<class T>
void Graph<T>::hashVertex(T vertex) {
    //A - 0, B - 1, C -2, etc.
    vertexMap.insert(std::make_pair(vertex, count));
    numVertices = adjVec.size();
    count++;
}

template<class T>
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
void Graph<T>::createAdj(int numV) {
    vector<int> emptyVector;
    for (int i = 0; i < numV; i++) {
        adjVec.push_back(emptyVector);
    }
    createReverseVertexMap();
}

template<class T>
void Graph<T>::populateAdj(T src, T dest) { //src: vertex, dest: edge to be added
    int srcKey = vertexMap.at(src); //hash index for src
    int destKey = vertexMap.at(dest); //hash index for dest
    (adjVec.at(srcKey)).push_back(destKey);
    (adjVec.at(destKey)).push_back(srcKey);
}

template<class T>
void Graph<T>::displayAdjVec() {
    cout << endl << "Adjacency List" << endl;
    for (unsigned int i = 0; i < adjVec.size(); i++) {
        cout << unHash(i) << "-> ";
        for (unsigned int j = 0; j < adjVec[i].size(); j++) {
            cout << unHash(adjVec[i][j]) << " ";
        }
        cout << endl;
    }
    cout << endl;
}

template<class T>
void Graph<T>::DFS(T node, ofstream& outputFile) {
    outputFile << endl <<  "-----------------------------------------------------------------------------------------------------"
               << endl << "DFS" << endl << endl;
    dfsVector.clear();
    int value = vertexMap.at(node);
    //cout << endl << "Node is: " << unHash(value) << endl;
    outputFile << "Node is: " << unHash(value) << endl;

    // Mark all the vertices as not visited
    bool *visited = new bool[adjVec.size()];
    for (unsigned int i = 0; i < adjVec.size(); i++) {
        visited[i] = false;
    }
    outputFile << "DFS traversal of " << node << " is: ";
    DFSHelper(value, visited, outputFile);
    createDFSEdgeList(outputFile);
}

template<class T>
void Graph<T>::DFSHelper(int v, bool visited[], ofstream& outputFile) {
    visited[v] = true;
    outputFile << unHash(v) << " ";
    dfsVector.push_back(v);
    // Recur for all the vertices adjacent to this vertex
    vector<int>::iterator i;
    for (i = adjVec[v].begin(); i != adjVec[v].end(); i++) {
        if (!visited[*i]) {
            DFSHelper(*i, visited, outputFile);
        }
    }
}

template<class T>
void Graph<T>::createDFSEdgeList(ofstream& outputFile) {
    dfsEdgeList.clear();
    dfsEdgeList.push_back(std::make_pair(dfsVector[0], dfsVector[1]));
    for (unsigned int i = 1; i < dfsVector.size(); i++) {
        dfsEdgeList.push_back(std::make_pair(dfsVector[i], dfsVector[i + 1]));
    }
    outputFile << endl << "Edge List: (";
    for (auto itr = dfsEdgeList.begin(); itr != dfsEdgeList.end() - 1; ++itr) {
        if (itr == dfsEdgeList.end() - 2) {
            outputFile << "{" << unHash(itr->first) << " - " << unHash(itr->second) << "}";
        } else {
            outputFile << "{" << unHash(itr->first) << " - " << unHash(itr->second) << "} ";
        }
    }
    outputFile << ")" << endl;
}

template<class T>
void Graph<T>::BFS(T node, ofstream& outputFile) {

    outputFile << endl <<  "-----------------------------------------------------------------------------------------------------"
               << endl <<"BFS" << endl;
    vector<int> bfsVector;
    int value = vertexMap.at(node);
    outputFile << endl << "Node is: " << unHash(value) << endl;
    bool *visited = new bool[adjVec.size()];
    for (unsigned int i = 0; i < adjVec.size(); i++) {
        visited[i] = false;
    }

    list<int> queue;
    visited[value] = true;
    queue.push_back(value);

    vector<int>::iterator i;

    outputFile << "BFS traversal of " << node << " is: ";
    while (!queue.empty()) {
        value = queue.front();
        bfsVector.push_back(value);
        outputFile << unHash(value) << " ";
        queue.pop_front();

        for (i = adjVec[value].begin(); i != adjVec[value].end(); i++) {
            if (!visited[*i]) {
                visited[*i] = true;
                queue.push_back(*i);
            }
        }
    }
    outputFile << endl;
    createBFSEdgeList(bfsVector, outputFile);
}

template<class T>
void Graph<T>::createBFSEdgeList(vector<int> bfsV, ofstream& outputFile) {

    vector<std::pair<int, int>> bfsEdgeList;

    bfsEdgeList.push_back(std::make_pair(bfsV[0], bfsV[1]));
    for (unsigned int i = 1; i < bfsV.size(); i++) {
        bfsEdgeList.push_back(std::make_pair(bfsV[i], bfsV[i + 1]));
    }

    outputFile << "Edge List: (";

    for (auto itr = bfsEdgeList.begin(); itr != bfsEdgeList.end() - 1; ++itr) {
        if (itr == bfsEdgeList.end() - 2) {
            outputFile << "{" << unHash(itr->first) << " - " << unHash(itr->second) << "}";
        } else {
            outputFile << "{" << unHash(itr->first) << " - " << unHash(itr->second) << "} ";
        }
    }

    outputFile << ")" << endl;
}

template<class T>
// Prints ALL paths from source to destination
int Graph<T>::getAllPaths(T source, T destination) {
    int start = vertexMap.at(source);
    int dest = vertexMap.at(destination);
    int size = 0;

    //keep track of which nodes visited
    bool *visited = new bool[adjVec.size()];

    // Array to store paths
    int *path = new int[adjVec.size()];
    int path_index = 0; // Initialize path[] as empty

    // Initialize all vertices as not visited
    for (unsigned int i = 0; i < adjVec.size(); i++){
        visited[i] = false;
    }

    // Call the helper function to print all paths
    cout << endl << source << "(" << start << ") -> " << destination << "(" << dest << ") paths are: " << endl;
    size = getAllPathsHelper(start, dest, visited, path, path_index);
    cout << "SIZE OF GET ALL PATHS: " << size << endl;
    return size;

}

template<class T>
int Graph<T>::getAllPathsHelper(int s, int d, bool *visited, int *path, int &path_index) {

    // Mark the current node and store in path[]
    visited[s] = true;
    path[path_index] = s;
    path_index++;

    vector<int> helperPaths;

    // If current vertex is same as destination, then print current path[]
    if (s == d) {
        for (int i = 0; i < path_index; i++) {
            helperPaths.push_back(path[i]);
            cout << path[i] << " ";
        }
        int pathSize = helperPaths.size();
        cout << "The size of this path is: " << pathSize << endl;
        return pathSize;
    } else { // If curr vertex is not destination, recur for all the vertices adjacent to current vertex
        vector<int>::iterator i;
        for (i = adjVec[s].begin(); i != adjVec[s].end(); ++i) {
            if (!visited[*i]) {
                getAllPathsHelper(*i, d, visited, path, path_index);
            }
        }
    }

    // Remove current vertex from path[] and mark it as unvisited
    path_index--;
    visited[s] = false;
    return 0;
}

/*
 * Make a Connection with the smallest number of introductions.
 * A set of introductions that need to be made in order for person A
 * to be introduced to person D.
 * Ex: {(A - B), (B - D)}
 */
template<class T>
void Graph<T>::makeConnection(T s, T d, ofstream& outputFile) {
    outputFile << endl <<  "-----------------------------------------------------------------------------------------------------"
               << endl << "Make Connection" << endl << endl;

    int src = vertexMap.at(s);
    int dest = vertexMap.at(d);

    outputFile << "Nodes to connect: " << unHash(src) << " and " << unHash(dest);
    printShortestDistance(src,dest, outputFile);
}

template<class T>
bool Graph<T>::connectionBFS(int src, int dest, int pred[], int dist[]) {

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

                if (adjVec[u][i] == dest){
                    return true;
                }
            }
        }
    }
    return false;
}

template<class T>
void Graph<T>::printShortestDistance(int s, int dest, ofstream& outputFile) {

    int pred[adjVec.size()], dist[adjVec.size()];
    vector<int> shortestPathVector;
    if (!connectionBFS(s, dest, pred, dist)) {
        outputFile << "Given source and destination cannot be connected";
        return;
    }

    int crawl = dest;
    shortestPathVector.push_back(crawl);
    while (pred[crawl] != -1) {
        shortestPathVector.push_back(pred[crawl]);
        crawl = pred[crawl];
    }

    // printing path from source to destination
    outputFile << "\nTo connect " << unHash(s) << " to " << unHash(dest) << ", the path is: " << endl;
    for (int i = shortestPathVector.size() - 1; i >= 0; i--) {
        if(i == 0){
            outputFile << unHash(shortestPathVector[i]) << "";
        }
        else{
            outputFile << unHash(shortestPathVector[i]) << " -> ";
        }
    }
    outputFile << endl;
    createEdgeList(shortestPathVector, outputFile);
}

template<class T>
void Graph<T>::createEdgeList(vector<int> path, ofstream& outputFile) {

    vector<std::pair<int, int>> edgeList;
    edgeList.clear();
    std::reverse(path.begin(), path.end());

    edgeList.push_back(std::make_pair(path[0], path[1]));

    for (unsigned int i = 1; i < path.size(); i++) {
        edgeList.push_back(std::make_pair(path[i], path[i + 1]));
    }
    outputFile << "Edge List: (";
    for (auto itr = edgeList.begin(); itr != edgeList.end() - 1; ++itr) {
        if (itr == edgeList.end() - 2) {
            outputFile << "{" << unHash(itr->first) << " - " << unHash(itr->second) << "}";
        } else {
            outputFile << "{" << unHash(itr->first) << " - " << unHash(itr->second) << "} ";
        }
    }
    outputFile << ")" << endl;
}

template<class T>
void Graph<T>::girvanNewman(ofstream& outputFile) {

    outputFile << endl <<  "-----------------------------------------------------------------------------------------------------"
               << endl << "Discovering Communities" << endl << endl;

    for(int i = 0; i < vertexMap.size(); i++){
        for(int j = i + 1; j < vertexMap.size(); j++){
            findShortestDistancesGirvan(i, j); //calculates betweenness as well
        }
    }

    //displayBetweennessMap();
    removeEdges();
    showCommunities(outputFile);
}

template<class T>
void Graph<T>::findShortestDistancesGirvan(int s, int dest) {

    // predecessor[i] contains parent of i and dist[] stores distance of i from source
    int parent[adjVec.size()], dist[adjVec.size()];
    vector<int> girvanPath;
    vector<std::pair<int, int>> edgeList;
    if (!BFSforGirvanAlgo(s, dest, parent, dist)) { //if no connection made
        cout << "Given source and destination cannot be connected";
        return;
    }

    int curr = dest;
    girvanPath.push_back(curr); //push curr to path
    while (parent[curr] != -1) {
        girvanPath.push_back(parent[curr]); //create vector path
        curr = parent[curr];
    }

    // printing path from source to destination
    cout << "\nTo connect " << unHash(s) << " to " << unHash(dest) << ", the path is: ";
    for (int i = girvanPath.size() - 1; i >= 0; i--) {
        if(i == 0){
            cout << unHash(girvanPath[i]) << "";
        }
        else{
            cout << unHash(girvanPath[i]) << " -> ";
        }

    }
    cout << endl;
    std::reverse(girvanPath.begin(), girvanPath.end());

    //create edgeList
    for (unsigned int i = 0; i < girvanPath.size(); i++) {
        edgeList.push_back(std::make_pair(girvanPath[i], girvanPath[i + 1]));
    }
    cout << "Shortest Path Edge List: (";
    for (auto itr = edgeList.begin(); itr != edgeList.end() - 1; ++itr) {
        if (itr == edgeList.end() - 2) {
            cout << "{" << unHash(itr->first) << " - " << unHash(itr->second) << "}";
        } else {
            cout << "{" << unHash(itr->first) << " - " << unHash(itr->second) << "} ";
        }
    }
    cout << ")" << endl;

    calculateBetweenness(edgeList);
}


template<class T>
bool Graph<T>::BFSforGirvanAlgo(int src, int dest, int *parent, int *distanceFromParent) {

    list<int> queue;

    //checks if nodes are visited
    bool visited[adjVec.size()];
    int shortestDistance;

    //setting initial values to false/infinity
    for (unsigned int i = 0; i < adjVec.size(); i++) {
        visited[i] = false;
        distanceFromParent[i] = INT_MAX;
        shortestDistance = INT_MAX;
        parent[i] = -1;
    }

    //visit source and set distance to 0
    visited[src] = true;
    distanceFromParent[src] = 0; //distance from root is 0
    queue.push_back(src);

    while (!queue.empty()) {
        int u = queue.front();
        if(GparentChildrenCount.find(u) != GparentChildrenCount.end()){
            GparentChildrenCount.insert(std::make_pair(u, 0));
        }

        vector<int> tempVec;
        parentsMap.insert(std::make_pair(u, tempVec));

        queue.pop_front();

        for (unsigned int i = 0; i < adjVec[u].size(); i++) {
            if (visited[adjVec[u][i]] == false) {
                visited[adjVec[u][i]] = true; //visit the node

                //check to see if child node is more than 1 level away from parent
                if(distanceFromParent[adjVec[u][i]] > distanceFromParent[u] + 1){
                    distanceFromParent[adjVec[u][i]] = distanceFromParent[u] + 1;
                    parent[adjVec[u][i]] = u;  //add Node current as a parent of Node neighbor
                }

                //check neighboring nodes, add to queue if not already in queue
                std::list<int>::iterator iter = std::find (queue.begin(), queue.end(), adjVec[u][i]);
                if(iter == queue.end()){
                    queue.push_back(adjVec[u][i]);
                }

                //keeping track of distance from roots
                if(adjVec[u][i] == dest){
                    if(distanceFromParent[adjVec[u][i]] <= shortestDistance) {
                        shortestDistance = distanceFromParent[adjVec[u][i]]; //update shortest distance
                        GparentChildrenCount[u]++; //incr the # of children for a vertex
                        //visited[adjVec[u][i]] = false; //reset to false so it can be visited again
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

template<class T>
void Graph<T>::calculateBetweenness(vector<std::pair<int, int>> edgeList) {

    for(int i = 0; i < edgeList.size()-1; i++){

        //make pairs of betweennessVal - Node & Node - betweennessVal
        std::pair<int,int> p1 = std::make_pair(edgeList[i].first,edgeList[i].second);
        std::pair<int,int> p2 = std::make_pair(edgeList[i].second,edgeList[i].first);
        std::map<std::pair<int,int>,int>::iterator it, it2;

        it = betweennessMap.find(p1);
        it2 = betweennessMap.find(p2);

        //if edge is encountered, incr count of it
        if((it != betweennessMap.end()) && (it2 != betweennessMap.end())){
            betweennessMap[edgeList[i]]++;
        }
        else{
            betweennessMap.emplace(std::make_pair(edgeList[i].first, edgeList[i].second),1);
        }
    }
}

template<class T>
void Graph<T>::displayBetweennessMap() {

    cout << endl << "Edge" << " - " << "Betweenness Value" << endl;
    for(auto it = betweennessMap.begin(); it != betweennessMap.end(); ++it){
        cout << unHash(it->first.first) << " " << unHash(it->first.second) << " - " << it->second << endl;
    }
    for(auto it = betweennessMap.begin(); it != betweennessMap.end(); ++it){
        multimap.emplace(it->second, it->first);
    }
    cout << endl << "Betweenness Value" << " - " << "Edge" << endl;
    for(auto it = multimap.begin(); it != multimap.end(); ++it){
        cout << it->first << " - " << unHash(it->second.first) << " " << unHash(it->second.second)  << '\n';
    }

}

template<class T>
void Graph<T>::removeEdges() {

    //displayAdjVec();

    int sum = 0;
    for(auto i = multimap.begin(); i != multimap.end(); i++){
        sum += i->first;
    }

    sum /= multimap.size();
    cout << sum << endl;
    double percentageToDelete = sum*(0.23);
    cout << percentageToDelete << endl;
    int deletedEdges = 0;
    vector<std::pair<int,int>> deletedEdgesVector;

    for(auto it = multimap.begin(); it != multimap.end(); it++){
        if(it->first >= percentageToDelete*2.5){
            typename std::unordered_map<int,T>::iterator vertex = reverseVertexMap.find(it->second.first);
            typename std::unordered_map<int,T>::iterator edge = reverseVertexMap.find(it->second.second);

            int x = vertex->first;
            int y = edge->first;

            vector<int>::reverse_iterator vertexIterator;
            vector<int>::reverse_iterator edgeIterator;

            for (vertexIterator = adjVec[x].rbegin(); vertexIterator < adjVec[x].rend(); vertexIterator++) {
                if (*vertexIterator == edge->first) {
                    adjVec[x].erase((vertexIterator + 1).base());
                    for(edgeIterator = adjVec[y].rbegin(); edgeIterator < adjVec[y].rend(); edgeIterator++){
                        if (*edgeIterator == vertex->first) {
                            adjVec[y].erase((edgeIterator + 1).base());
                        }
                    }
                }
            }
            deletedEdges++;
            deletedEdgesVector.push_back(std::make_pair(vertex->first,edge->first));
        }
    }
    // displayAdjVec();
}

template<class T>
vector<int> Graph<T>::BFSPostEdgeDeletion(T node) {

    vector<int> bfsVector;
    int value = vertexMap.at(node);
    bool *visited = new bool[adjVec.size()];
    for (unsigned int i = 0; i < adjVec.size(); i++) {
        visited[i] = false;
    }

    list<int> queue;
    visited[value] = true;
    queue.push_back(value);

    vector<int>::iterator i;

    while (!queue.empty()) {
        value = queue.front();
        bfsVector.push_back(value);
        queue.pop_front();

        for (i = adjVec[value].begin(); i != adjVec[value].end(); i++) {
            if (!visited[*i]) {
                visited[*i] = true;
                queue.push_back(*i);
            }
        }
    }
    sort(bfsVector.begin(),bfsVector.end());
    return bfsVector;
}

template<class T>
void Graph<T>::showCommunities(ofstream& outputFile) {

    std::map<vector<int>, vector<int>>::iterator it;

    for(int i = 0; i < adjVec.size(); i++){
        it = bigBoiMap.find(BFSPostEdgeDeletion(unHash(i)));
        if(it != bigBoiMap.end()){
            it->second.push_back(i);
        }
        else{
            vector<int> tempVec;
            tempVec.push_back(i);
            vector<int> t = BFSPostEdgeDeletion(unHash(i));
            bigBoiMap.insert(std::pair<vector<int>,vector<int>>(t, tempVec));
        }
    }

    outputFile << "The communities are: " << endl << endl;
    int i = 1;

    for(it = bigBoiMap.begin(); it != bigBoiMap.end(); it++){
        outputFile << "Community " << i << endl << "Size: " << it->first.size() << endl;
        outputFile << "Members: ";
        sort(it->second.begin(), it->second.end());
        for(auto i = it->second.begin(); i != it->second.end(); i++){
            outputFile << unHash(*i) << " ";
        }
        i++;
        outputFile << endl << endl;
    }
}

template<class T>
void Graph<T>::displayBigBoyMap() {
    std::map<vector<int>, vector<int>>::iterator it2;
    for(it2 = bigBoiMap.begin(); it2 != bigBoiMap.end(); it2++){
        cout << "PATH: ";
        for(auto i = it2->first.begin(); i != it2->first.end(); i++){
            cout << *i << " -> ";
            for(auto j = it2->second.begin(); j != it2->second.end(); j++){
                cout << "Nodes of the path: " << *j << " ";
            }
            cout << endl;
        }
    }
}

// utility function to check if current vertex is already present in path
template<class T>
int Graph<T>::isNotVisited(int x, vector<int> &path) {
    int size = path.size();
    for (int i = 0; i < size; i++)
        if (path[i] == x)
            return 0;
    return 1;
}

// utility function for printing found path in graph
template<class T>
void Graph<T>::printPath(vector<int> &path) {
    int size = path.size();
    cout << "the path is: ";
    for (int i = 0; i < size; i++) {
        cout << path[i] << " ";
    }
    cout << "The size of this path is: " << size << ".";
}

template<class T>
void Graph<T>::girvanDFS(int src, int destination) {
    vector<int> s; //stack
    vector<int> n[adjVec.size()]; //adjacent nodes
    bool visited[adjVec.size()]; //keep track of visited

    if (src == destination)//if we found  destination
    {
        //print all the nodes from bottom to top of stack
        for (int i = 0; i < s.size(); i++){
            printf("%d ", s[i]);
        }

        printf("%d\n", destination);
        return;
    }

    visited[src] = true;
    s.push_back(src);

    for (int i = 0; i < n[src].size(); i++) {
        int beta = n[src][i];
        if (!visited[beta]) {
            girvanDFS(beta, destination);
        }
    }

    visited[src] = false;
    s.pop_back();
}


template<class T>
//helper function to print any map
void Graph<T>::printMaps() {
    for(int i = 0; i < GparentChildrenCount.size(); i++){
        cout << "u: " << unHash(i) << " " << GparentChildrenCount[i]  << endl;
    }
    cout << endl;
}

template<class T>
void Graph<T>::findNumEdges(T ctr, T cting) {

    int connector = vertexMap.at(ctr);
    int connecting = vertexMap.at(cting);

    vector<bool> visited(adjVec.size(), 0);

    // Initialize distances as 0
    vector<int> distance(adjVec.size(), 0);

    // queue to do BFS
    std::queue<int> Q;
    distance[connector] = 0;

    Q.push(connector);
    visited[connector] = true;
    while (!Q.empty()) {
        int x = Q.front();
        Q.pop();

        for (int i = 0; i < adjVec[x].size(); i++) {
            if (visited[adjVec[x][i]]) {
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

    isConnected(connector, connecting, discovered, edgePath);
    cout << "The edge path from " << unHash(connector) << " to " << unHash(connecting) << " is: ";
    for (int i: edgePath)
        cout << unHash(i) << ' ';
    cout << endl;
}

// helper function to find total number of edges
template<class T>
bool Graph<T>::isConnected(int src, int dest, vector<bool> &discovered, vector<int> &path) {

    discovered[src] = true; // mark current  as discovered
    path.push_back(src); //add curr  to path

    if (src == dest) { // if destination is found
        return true;
    }

    for (int i: adjVec[src]) { // perform for every edge in adjList
        if (!discovered[i]) { //recur if u is not discovered
            if (isConnected(i, dest, discovered, path))
                return true;
        }
    }
    path.pop_back(); // backtrack: remove current node from the path
    return false; //false ret if dest is not reachable
}

template<class T>
T Graph<T>::getKey(T value) {
    typename std::unordered_map<int, T>::iterator it;
    T key;
    for (it = vertexMap.begin(); it != vertexMap.end(); ++it) {
        if (it->second == value) {
            key = it->first;
            break;
        }
    }
    return key;
}


template<class T>
void Graph<T>::calculateBetweenness(int parents[]) {

    for(int i = 0; i < sizeof(parents); i++){
        cout << parents[i] << endl;
    }
    cout << endl;
    int sum = 0, averageChildren = 0;
    for(auto i = GparentChildrenCount.begin(); i != GparentChildrenCount.end(); i++){
        cout << "Vertex " << i->first << ": " << i->second << endl;
        sum += i->second;
    }
    averageChildren = sum / GparentChildrenCount.size();
    cout << "size of map: " << GparentChildrenCount.size() << " avg children: " << averageChildren << endl;

    for(auto i = GparentChildrenCount.begin(); i != GparentChildrenCount.end(); i++){
        if(i->second >= averageChildren){
            removeNode(i->first);
        }
    }
    cout << "size of map after deletion: " << GparentChildrenCount.size() << endl;
}

#endif //INC_20S_3353_PA02_GRAPH_H