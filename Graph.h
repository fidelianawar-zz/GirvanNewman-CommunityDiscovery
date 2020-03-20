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

    vector<vector<int>> adjVec;
    vector<vector<int>> communitiesAdjVec;
    std::unordered_map<T, int> vertexMap;
    std::unordered_map<int, T> reverseVertexMap;
    std::map<std::pair<int,int>,int> betweennessMap;
    std::multimap<int,std::pair<int,int>> multimap;
    std::map<vector<int>, vector<int>> bigBoiMap;

    int count = 0;
    int numVertices;

    vector<int> dfsVector;
    vector<std::pair<int, int>> dfsEdgeList;

    vector<int> edgePath;
    std::unordered_map<int, int> GparentChildrenCount;
    std::unordered_map<int, vector<int>> parentsMap;


public:

    Graph();
    T unHash(int);
    void createReverseVertexMap();
    void populateAdj(T src, T dest);
    void displayAdjVec();
    void hashVertex(T);
    void createAdj(int);

    void DFS(T, ofstream& outputFile);
    void DFSHelper(int v, bool visited[], ofstream& outputFile);
    void createDFSEdgeList(ofstream& outputFile);
    void BFS(T, ofstream& outputFile);
    void createBFSEdgeList(vector<int>, ofstream& outputFile);
    void createEdgeList(vector<int>, ofstream&); //made separate edge lists for formatting issues

    void makeConnection(T, T, ofstream&);
    bool connectionBFS(int src, int dest, int pred[], int dist[]);
    void printShortestDistance(int s, int dest, ofstream&);

    void girvanNewman(ofstream&);
    bool BFSforGirvanAlgo(int src, int dest, int parent[], int distanceFromParent[]);
    void printShortestDistanceGirvan(int s, int dest);
    void calculateBetweenness(vector<std::pair<int, int>>);
    void calculateBetweennessv2();
    void displayBetweennessMap();
    void removeEdges();
    vector<int> BFSPostEdgeDeletion(T);
    void showCommunities(ofstream&);
    void displayBigBoyMap();

    void findNumEdges(T, T);
    void printMaps();
    T getKey(T value);
    int getAllPaths(T s, T d);
    int getAllPathsHelper(int u, int d, bool *visited, int *path, int &path_index);
    void girvanDFS(int,int);
    bool isConnected(int src, int dest, vector<bool> &discovered, vector<int> &path);
    int isNotVisited(int x, vector<int> &path);
    int findPaths(vector<vector<int> > &g, int src, int dst);
    void printPath(vector<int> &);

};

template<class T>
Graph<T>::Graph() {
    cout << "Making new graph..............";
}

template<class T>
void Graph<T>::hashVertex(T vertex) { //hashes T values to an index in map
    //A - 0, B - 1, C -2, etc.
    vertexMap.insert(std::make_pair(vertex, count));
    numVertices = adjVec.size();
    count++;
}

template<class T>
void Graph<T>::createReverseVertexMap() { //creates a reverse key/value map of original
    //0 - A, 1 - B, 2 - C, etc.
    for (auto itr = vertexMap.begin(); itr != vertexMap.end(); ++itr) {
        reverseVertexMap[itr->second] = itr->first;
    }
}

template<class T>
T Graph<T>::unHash(int vertex) { //used to obtain real T value
    return reverseVertexMap[vertex];
}

template<class T>
void Graph<T>::createAdj(int numV) { //create empty list with empty vector
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
void Graph<T>::displayAdjVec() { //see contents of adjacency list
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

    //if supplied node does not exist in vertex list
    if(vertexMap.find((node)) == vertexMap.end()){
        outputFile << "The node does not exist." << endl;
        return;
    }


    dfsVector.clear(); //reset for each new iteration
    int value = vertexMap.at(node); //obtain hashed value
    outputFile << "Node is: " << unHash(value) << endl;

    // Mark all the vertices as not visited
    bool *visited = new bool[adjVec.size()];
    for (unsigned int i = 0; i < adjVec.size(); i++) {
        visited[i] = false;
    }

    outputFile << "DFS traversal of " << node << " is: ";
    DFSHelper(value, visited, outputFile);
    createDFSEdgeList(outputFile); //create edge list from returned path
}

template<class T>
void Graph<T>::DFSHelper(int v, bool visited[], ofstream& outputFile) {
    visited[v] = true; //visit node
    outputFile << unHash(v) << " ";
    dfsVector.push_back(v); //push back node to vector
    vector<int>::iterator i;
    for (i = adjVec[v].begin(); i != adjVec[v].end(); i++) {
        if (!visited[*i]) {
            DFSHelper(*i, visited, outputFile); //perform DFS on all nodes from original
        }
    }
}

template<class T>
void Graph<T>::createDFSEdgeList(ofstream& outputFile) {

    dfsEdgeList.clear(); //make sure list is cleared

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
               << endl << "BFS" << endl;

    //if node not found, return from function
    if(vertexMap.find((node)) == vertexMap.end()){
        outputFile << "This node does not exist." << endl;
        return;
    }

    vector<int> bfsVector; //create temp vector
    int value = vertexMap.at(node); //hash to get integer representation of T value
    outputFile << endl << "Node is: " << unHash(value) << endl;
    bool *visited = new bool[adjVec.size()]; //visiting array
    for (unsigned int i = 0; i < adjVec.size(); i++) {
        visited[i] = false; //set all nodes as unvisited
    }

    list<int> queue;
    visited[value] = true; //visit node and push to queue
    queue.push_back(value);

    vector<int>::iterator i;

    outputFile << "BFS traversal of " << node << " is: ";
    while (!queue.empty()) { //iterate until all nodes traversed
        value = queue.front();
        bfsVector.push_back(value);
        outputFile << unHash(value) << " ";
        queue.pop_front(); //remove from queue

        //visit all adjacent nodes of vertex
        for (i = adjVec[value].begin(); i != adjVec[value].end(); i++) {
            if (!visited[*i]) {
                visited[*i] = true; //if unvisisted, visit, mark as visited, and push to queue
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

    //if either node not found, return from func
    if(vertexMap.find((s)) == vertexMap.end() || vertexMap.find((d)) == vertexMap.end()){
        outputFile << "This source/destination node does not exist." << endl;
        return;
    }

    int src = vertexMap.at(s); //hash to get index/integer representation
    int dest = vertexMap.at(d);

    outputFile << "Nodes to connect: " << unHash(src) << " and " << unHash(dest);
    printShortestDistance(src,dest, outputFile);
}

template<class T>
void Graph<T>::printShortestDistance(int s, int dest, ofstream& outputFile) {

    // predecessor[i] contains parent of i and dist[] stores distance of i from source
    int pred[adjVec.size()], dist[adjVec.size()];
    vector<int> shortestPathVector; //keep track of shortest paths
    if (!connectionBFS(s, dest, pred, dist)) {
        outputFile << "Given source and destination cannot be connected";
        return;
    }

    int d = dest;
    shortestPathVector.push_back(d);
    while (pred[d] != -1) { //used for printing
        shortestPathVector.push_back(pred[d]);
        d = pred[d];
    }

    // printing path from source to destination
    outputFile << "\nTo travel from " << unHash(s) << " to " << unHash(dest) << ", the path is: " << endl;
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
bool Graph<T>::connectionBFS(int src, int dest, int pred[], int dist[]) {

    list<int> queue; //queue for BFS
    bool visited[adjVec.size()]; //checks if nodes are visited

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
        //for each adjacent node of vertex
        for (unsigned int i = 0; i < adjVec[u].size(); i++) {
            if (visited[adjVec[u][i]] == false) {
                visited[adjVec[u][i]] = true; //visit node
                dist[adjVec[u][i]] = dist[u] + 1; //update distance from root
                pred[adjVec[u][i]] = u; //save parent of node
                queue.push_back(adjVec[u][i]); //add to queue

                //stop when destination is found
                if (adjVec[u][i] == dest){
                    return true;
                }
            }
        }
    }
    return false;
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
    for(unsigned int i = 0; i < vertexMap.size(); i++){
        for(unsigned int j = i + 1; j < vertexMap.size(); j++){
            printShortestDistanceGirvan(i,j);
        }
    }
    displayBetweennessMap(); //displays edges with highest betweenness
    removeEdges(); //removes those within certain % of highest betweenness
    showCommunities(outputFile); //displays communities as a result of removing edges
}

template<class T>
void Graph<T>::printShortestDistanceGirvan(int s, int dest) {

    // predecessor[i] contains parent of i and dist[] stores distance of i from source
    int parent[adjVec.size()], dist[adjVec.size()];
    vector<int> girvanPath;
    vector<std::pair<int, int>> edgeList;
    if (!BFSforGirvanAlgo(s, dest, parent, dist)) {
        cout << "Given source and destination cannot be connected";
        return;
    }

    int curr = dest;
    girvanPath.push_back(curr);
    while (parent[curr] != -1) {
        girvanPath.push_back(parent[curr]);
        curr = parent[curr];
    }

    std::reverse(girvanPath.begin(), girvanPath.end()); //reverse to get correct order

    for (unsigned int i = 0; i < girvanPath.size(); i++) {
        edgeList.push_back(std::make_pair(girvanPath[i], girvanPath[i + 1]));
    }

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

    for(unsigned int i = 0; i < edgeList.size()-1; i++){

        //store values of edgeList to search for later
        std::pair<int,int> p1 = std::make_pair(edgeList[i].first,edgeList[i].second);
        std::pair<int,int> p2 = std::make_pair(edgeList[i].second,edgeList[i].first);
        std::map<std::pair<int,int>,int>::iterator it, it2;

        it = betweennessMap.find(p1); //use to see if it exists in map or not
        it2 = betweennessMap.find(p2); //if it doesn't, returns end of map

        //edge found in map - increment it's count
        if((it != betweennessMap.end()) && (it2 != betweennessMap.end())){
            betweennessMap[edgeList[i]]++;
        }
        else{ //if edge was not found in map, add it to it and incr count of edge
            betweennessMap.emplace(std::make_pair(edgeList[i].first, edgeList[i].second),1);
        }
    }
}

template<class T>
void Graph<T>::displayBetweennessMap() {

    //cout << endl << "Edge" << " - " << "Betweenness Value" << endl;
    for(auto it = betweennessMap.begin(); it != betweennessMap.end(); ++it){
        //cout << unHash(it->first.first) << " " << unHash(it->first.second) << " - " << it->second << endl;
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

    communitiesAdjVec = adjVec; //create tempVec so original adjVec can be used later

    int sum = 0;
    for(auto i = multimap.begin(); i != multimap.end(); i++){
        sum += i->first;
    }

    //calculation to know which edges to remove
    sum /= multimap.size();
    double percentageToDelete = sum*(0.15);
    int deletedEdges = 0;
    vector<std::pair<int,int>> deletedEdgesVector;

    for(auto it = multimap.begin(); it != multimap.end(); it++){
        if(it->first >= percentageToDelete*2.67){

            //find vertex/edge to delete later from adjVec
            typename std::unordered_map<int,T>::iterator vertex = reverseVertexMap.find(it->second.first);
            typename std::unordered_map<int,T>::iterator edge = reverseVertexMap.find(it->second.second);

            int x = vertex->first;
            int y = edge->first;

            vector<int>::reverse_iterator vertexIterator;
            vector<int>::reverse_iterator edgeIterator;

            //make sure to delete at u, v and v, u
            for (vertexIterator = communitiesAdjVec[x].rbegin(); vertexIterator < communitiesAdjVec[x].rend(); vertexIterator++) {
                if (*vertexIterator == edge->first) {
                    communitiesAdjVec[x].erase((vertexIterator + 1).base());
                    for(edgeIterator = communitiesAdjVec[y].rbegin(); edgeIterator < communitiesAdjVec[y].rend(); edgeIterator++){
                        if (*edgeIterator == vertex->first) {
                            communitiesAdjVec[y].erase((edgeIterator + 1).base());
                        }
                    }
                }
            }
            deletedEdges++;
            deletedEdgesVector.push_back(std::make_pair(vertex->first,edge->first));
        }
    }
    //displayAdjVec();
}

template<class T>
void Graph<T>::showCommunities(ofstream& outputFile) {

    std::map<vector<int>, vector<int>>::iterator it;

    //perform another BFS on leftover nodes to find which ones have same paths
    for(unsigned int i = 0; i < communitiesAdjVec.size(); i++){

        it = bigBoiMap.find(BFSPostEdgeDeletion(unHash(i)));  //store the path in map as key
        if(it != bigBoiMap.end()){ //if path exists, append node as value to path
            it->second.push_back(i);
        }

        else{ //if path does not exist, create new element in map with path and node
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

        sort(it->second.begin(), it->second.end()); //sort for alphabetical order

        for(auto i = it->second.begin(); i != it->second.end(); i++){
            outputFile << unHash(*i) << " ";
        }

        i++;
        outputFile << endl << endl;
    }
    //displayBigBoyMap();
}

template<class T>
vector<int> Graph<T>::BFSPostEdgeDeletion(T node) {

    //same as previous BFS except w/o extra formatting and tracking of parents/distance from root
    vector<int> bfsVector;
    int value = vertexMap.at(node);
    bool *visited = new bool[communitiesAdjVec.size()];
    for (unsigned int i = 0; i < communitiesAdjVec.size(); i++) {
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

        for (i = communitiesAdjVec[value].begin(); i != communitiesAdjVec[value].end(); i++) {
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
// utility function for finding paths in graph from source to destination
int Graph<T>::findPaths(vector<vector<int> > &g, int src, int dst) { //v = adjVec.size(), create a queue which stores the paths
    std::queue<vector<int> > q; // path vector to store the current path
    vector<int> path;
    path.push_back(src);
    q.push(path);
    while (!q.empty()) {
        path = q.front();
        q.pop();
        int last = path[path.size() - 1]; // if last vertex is the desired destination, then print the path
        if (last == dst) {
            printPath(path);
            return path.size();
        } // traverse to all the nodes connected to current vertex and push new path to queue
        for (int i = 0; i < g[last].size(); i++) {
            if (isNotVisited(g[last][i], path)) {
                vector<int> newpath(path);
                newpath.push_back(g[last][i]);
                q.push(newpath);
            }
        }
    }
}

template<class T>
void Graph<T>::girvanDFS(int src, int destination) {

    vector<int> s; //stack
    vector<int> g[adjVec.size()]; //stores adjacent elements of vertex
    bool visited[adjVec.size()]; //keep track of visited nodes

    if (src == destination){ //if we found our destination
        for (int i = 0; i < s.size(); i++){
            printf("%d ", s[i]); //print nodes from top of stack to bottom
        }

        printf("%d\n", destination);
        return;
    }

    visited[src] = true;
    s.push_back(src); //push to path

    for (int i = 0; i < g[src].size(); i++) {
        int beta = g[src][i];
        if (!visited[beta]) {
            girvanDFS(beta, destination);
        }
    }

    visited[src] = false;
    s.pop_back();
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
// Prints all paths from 's' to 'd'
int Graph<T>::getAllPaths(T source, T destination) {

    int start = vertexMap.at(source);
    int dest = vertexMap.at(destination);
    int size = 0;

    bool *visited = new bool[adjVec.size()]; //mark all the vertices as not visited
    int *path = new int[adjVec.size()];  //create array to store paths
    int path_index = 0; //initialize path as empty


    for (unsigned int i = 0; i < adjVec.size(); i++){
        visited[i] = false; // Initialize all vertices as not visited
    }

    cout << endl << source << "(" << start << ") -> " << destination << "(" << dest << ") paths are: " << endl;
    size = getAllPathsHelper(start, dest, visited, path, path_index);
    cout << "SIZE OF GET ALL PATHS: " << size << endl;
    return size;
}

template<class T>
int Graph<T>::getAllPathsHelper(int u, int d, bool *visited, int *path, int &path_index) {

    //Mark the current node and store in path
    visited[u] = true;
    path[path_index] = u;
    path_index++;

    vector<int> helperPaths;

    //if curr = dest, print path
    if (u == d) {
        for (int i = 0; i < path_index; i++) {
            helperPaths.push_back(path[i]);
            cout << path[i] << " ";
        }
        int pathSize = helperPaths.size();
        cout << "The size of this path is: " << pathSize << endl;
        return pathSize;
    } else { // If curr vertex is not destination, recur for all the vertices adjacent to current vertex
        vector<int>::iterator i;
        for (i = adjVec[u].begin(); i != adjVec[u].end(); ++i) {
            if (!visited[*i]) {
                getAllPathsHelper(*i, d, visited, path, path_index);
            }
        }
    }
    // Remove current vertex from path and mark it as unvisited
    path_index--;
    visited[u] = false;
    return 0;
}

template<class T>
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

    // queue to do BFS.
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

template<class T>
bool Graph<T>::isConnected(int src, int dest, vector<bool> &discovered, vector<int> &path) {

    discovered[src] = true; //mark as visited
    path.push_back(src); //add current node to path

    if (src == dest) { // if destination vertex is found
        return true;
    }

    for (int i: adjVec[src]) { //perform for every edge of vertex
        if (!discovered[i]) { //recur if not found
            if (isConnected(i, dest, discovered, path))
                return true;
        }
    }
    path.pop_back();
    return false; //error check if dest is not reachable from source
}

template<class T>
void Graph<T>::calculateBetweennessv2() {

    int sum = 0, averageChildren = 0;
    for(auto i = GparentChildrenCount.begin(); i != GparentChildrenCount.end(); i++){
        cout << "Vertex " << i->first << ": " << i->second << endl;
        sum += i->second;
    }
    averageChildren = sum / GparentChildrenCount.size();
    cout << "size of map: " << GparentChildrenCount.size() << " avg children: " << averageChildren << endl;

    for(auto i = GparentChildrenCount.begin(); i != GparentChildrenCount.end(); i++){
        if(i->second >= averageChildren){
            //removeNode(i->first);
        }
    }
    cout << "size of map after deletion: " << GparentChildrenCount.size() << endl;
}

#endif //INC_20S_3353_PA02_GRAPH_H