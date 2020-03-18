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
class InfoTracker {
    bool visited;
    int distanceFromOrigin;
    T parent;
};

class graphNode {
public:
    int id;
    string name;
    bool status;
    double weight;
};


template<class T>
class Graph {

    int numVertices;

    vector<vector<int>> adjVec;
    vector<int> numPathsSize;
    vector<vector<int>> allNumShortestPaths;
    vector<vector<int>> allPathsBetweenNodes;
    std::unordered_map<T, int> vertexMap;
    std::unordered_map<int, T> reverseVertexMap;
    int count = 0;

    vector<int> bfsVector;
    vector<int> dfsVector;
    vector<int> edgePath;
    vector<int> GirvanShortestPathVector;
    std::unordered_map<int, int> GparentChildrenCount;
    std::unordered_map<int, vector<int>> parentsMap;


    vector<std::pair<int, int>> bfsEdgeList;
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

    void makeConnection(T, T);

    void printShortestDistance(int s, int dest);
    void printShortestDistanceGirvan(int s, int dest);

    void printMaps();

    T getKey(T value);

    int getAllPaths(T s, T d);

    int getAllPathsHelper(int u, int d, bool *visited, int *path, int &path_index);

    void printAllPaths();

    void DFS(T);

    void DFSHelper(int v, bool visited[]);

    void BFS(T);

    bool connectionBFS(int src, int dest, int pred[], int dist[]);

    bool isConnected(int src, int dest, vector<bool> &discovered, vector<int> &path);

    void createEdgeList(vector<int>);

    void createGirvanShortestPathEdgeList();

    void createBFSEdgeList();

    void createDFSEdgeList();

    void girvanNewman1();

    void removeNode(int); //remove edges based on betweenness value

    int isNotVisited(int x, vector<int> &path);

    int findpaths(vector<vector<int> > &g, int src,
                  int dst);

    void printpath(vector<int> &);

    void girvanNewman();

    void girvanNewman2();

    //void ModifiedBFS(T, T);

    bool BFSforGirvan(int src, int dest, int parent[], int distanceFromParent[]);
    void printShortestDistanceG(int s, int dest);

    void girvanDFS(int,int);

    void girvan();

    void calculateBetweenness(int[]);

};

bool isNotVisited(int &i, vector<int> vector);

void printpath(vector<int> vector);

template<class T>
Graph<T>::Graph() {
    cout << "making a graph";
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
    cout << "number of vertices is: " << numV << endl;
    vector<int> emptyVector;
    for (int i = 0; i < numV; i++) {
        adjVec.push_back(emptyVector);
    }
    createReverseVertexMap();
    cout << endl;
}

template<class T>
void Graph<T>::populateAdj(T src, T dest) { //src: vertex, dest: edge to be added
    int srcKey = vertexMap.at(src); //hash index for src
    int destKey = vertexMap.at(dest); //hash index for dest
    //cout << "src key : " << unHash(srcKey) << " " << "dest key: " << unHash(destKey) << " " << endl;
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
void Graph<T>::DFS(T node) {
    dfsVector.clear();
    int value = vertexMap.at(node);
    cout << endl << "Node is: " << unHash(value) << endl;

    // Mark all the vertices as not visited
    bool *visited = new bool[adjVec.size()];
    for (unsigned int i = 0; i < adjVec.size(); i++) {
        visited[i] = false;
    }

    // Call the recursive helper function to print DFS traversal
    cout << "DFS traversal of " << node << " is: ";
    DFSHelper(value, visited);
    createDFSEdgeList();
}

template<class T>
void Graph<T>::DFSHelper(int v, bool visited[]) {
    visited[v] = true;
    cout << unHash(v) << " ";
    dfsVector.push_back(v);
    // Recur for all the vertices adjacent to this vertex
    vector<int>::iterator i;
    for (i = adjVec[v].begin(); i != adjVec[v].end(); i++) {
        if (!visited[*i]) {
            DFSHelper(*i, visited);
        }
    }
}

template<class T>
void Graph<T>::createDFSEdgeList() {
    dfsEdgeList.clear();
    dfsEdgeList.push_back(std::make_pair(dfsVector[0], dfsVector[1]));
    for (unsigned int i = 1; i < dfsVector.size(); i++) {
        dfsEdgeList.push_back(std::make_pair(dfsVector[i], dfsVector[i + 1]));
    }
    cout << endl << "DFS Edge List: (";
    for (auto itr = dfsEdgeList.begin(); itr != dfsEdgeList.end() - 1; ++itr) {
        if (itr == dfsEdgeList.end() - 2) {
            cout << "{" << unHash(itr->first) << " - " << unHash(itr->second) << "}";
        } else {
            cout << "{" << unHash(itr->first) << " - " << unHash(itr->second) << "} ";
        }
    }
    cout << ")" << endl;
}

template<class T>
void Graph<T>::BFS(T node) {
    bfsVector.clear();
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
    for (auto itr = bfsEdgeList.begin(); itr != bfsEdgeList.end() - 1; ++itr) {
        if (itr == bfsEdgeList.end() - 2) {
            cout << "{" << unHash(itr->first) << " - " << unHash(itr->second) << "}";
        } else {
            cout << "{" << unHash(itr->first) << " - " << unHash(itr->second) << "} ";
        }
    }
    cout << ")" << endl;
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
void Graph<T>::girvanNewman1() {
    cout << endl;

    for (unsigned int i = 0; i < adjVec.size(); i++) {
        vector<int> numPathSize;
        for (unsigned int j = 0; j < adjVec.size(); j++) {
            numPathSize.push_back(getAllPaths(unHash(i), unHash(j)));
            cout << getAllPaths(unHash(i), unHash(j)) << " ";
        }
        allPathsBetweenNodes.push_back(numPathSize);
        cout << "the size of numPathsSize at: " << numPathSize.size() << endl;
        numPathSize.clear();
    }
    cout << endl << "size of allPathsBetweenNodes: " << allPathsBetweenNodes.size() << endl << endl;

//    vector<int>::iterator i;
//    vector<vector<int>>::iterator j;

    //cout << endl << endl;
    for (int i = 0; i < allPathsBetweenNodes.size(); i++) {
        for (int j = 0; j < allPathsBetweenNodes.size(); j++) {
            //cout << allPathsBetweenNodes[i][j] << " ";
        }
        // cout << endl;
    }
}

template<class T>
// Prints all paths from 's' to 'd'
int Graph<T>::getAllPaths(T source, T destination) {
    int start = vertexMap.at(source);
    int dest = vertexMap.at(destination);
    int size = 0;

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
    size = getAllPathsHelper(start, dest, visited, path, path_index);
    cout << "SIZE OF GET ALL PATHS: " << size << endl;
    return size;

}

template<class T>
int Graph<T>::getAllPathsHelper(int u, int d, bool *visited, int *path, int &path_index) {
    // Mark the current node and store it in path[]
    visited[u] = true;
    path[path_index] = u;
    path_index++;

    vector<int> helperPaths;
    // If current vertex is same as destination, then print current path[]
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
    // Remove current vertex from path[] and mark it as unvisited
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
    // visited[n] for keeping track of visited
    // node in BFS
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

// Function to perform DFS traversal in a directed graph to find the
// complete path between source and destination vertices
template<class T>
bool Graph<T>::isConnected(int src, int dest, vector<bool> &discovered, vector<int> &path) {
    // mark current node as discovered
    discovered[src] = true;

    // include current node in the path
    path.push_back(src);

    // if destination vertex is found
    if (src == dest) {
        return true;
    }
    // do for every edge (src -> i)
    for (int i: adjVec[src]) {
        // u is not discovered
        if (!discovered[i]) {
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

                //stop when destination is found
                if (adjVec[u][i] == dest)
                    return true;
            }
        }
    }
    return false;
}

template<class T>
void Graph<T>::printShortestDistance(int s, int dest) {
    // predecessor[i] contains parent of i and dist[] stores distance of i from source
    int pred[adjVec.size()], dist[adjVec.size()];
    vector<int> shortestPathVector;
    if (!connectionBFS(s, dest, pred, dist)) {
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
    cout << "\nTo travel from " << unHash(s) << " to " << unHash(dest) << ", the path is: " << endl;
    for (int i = shortestPathVector.size() - 1; i >= 0; i--) {
        cout << unHash(shortestPathVector[i]) << " ";
    }
    cout << endl << endl;
    createEdgeList(shortestPathVector);
}

template<class T>
void Graph<T>::createEdgeList(vector<int> path) {

    vector<std::pair<int, int>> edgeList;
    edgeList.clear();
    std::reverse(path.begin(), path.end());

    edgeList.push_back(std::make_pair(path[0], path[1]));

    for (unsigned int i = 1; i < path.size(); i++) {
        edgeList.push_back(std::make_pair(path[i], path[i + 1]));
    }
    cout << "Edge List: (";
    for (auto itr = edgeList.begin(); itr != edgeList.end() - 1; ++itr) {
        if (itr == edgeList.end() - 2) {
            cout << "{" << unHash(itr->first) << " - " << unHash(itr->second) << "}";
        } else {
            cout << "{" << unHash(itr->first) << " - " << unHash(itr->second) << "} ";
        }
    }
    cout << ")" << endl;
}

template<class T>
void Graph<T>::girvanNewman() {
    cout << "inside of GirvanNewman" << endl;
    for(int i = 0; i < vertexMap.size(); i++){
        for(int j = i + 1; j < vertexMap.size(); j++){
            printShortestDistanceGirvan(i,j);
        }
    }
}

template<class T>
void Graph<T>::printShortestDistanceGirvan(int s, int dest) {

    // predecessor[i] contains parent of i and dist[] stores distance of i from source
    int pred[adjVec.size()], dist[adjVec.size()];
    vector<int> girvanPath;
    vector<std::pair<int, int>> edgeList;
    if (!connectionBFS(s, dest, pred, dist)) {
        cout << "Given source and destination cannot be connected";
        return;
    }

    int crawl = dest;
    girvanPath.push_back(crawl);
    while (pred[crawl] != -1) {
        girvanPath.push_back(pred[crawl]);
        crawl = pred[crawl];
    }

    // distance from source is in distance array
    cout << endl << "Girvan shortest path length is : " << dist[dest];

    // printing path from source to destination
    cout << "\nTo travel from " << unHash(s) << " to " << unHash(dest) << ", the path is: ";
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
    edgeList.push_back(std::make_pair(girvanPath[0], girvanPath[1]));

    for (unsigned int i = 1; i < girvanPath.size(); i++) {
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
}

// utility function to check if current
// vertex is already present in path
template<class T>
int Graph<T>::isNotVisited(int x, vector<int> &path) {
    int size = path.size();
    for (int i = 0; i < size; i++)
        if (path[i] == x)
            return 0;
    return 1;
}

// utility function for printing
// the found path in graph
template<class T>
void Graph<T>::printpath(vector<int> &path) {
    int size = path.size();
    cout << "the path is: ";
    for (int i = 0; i < size; i++) {
        cout << path[i] << " ";
    }
    cout << "The size of this path is: " << size << ".";
}

template<class T>
// utility function for finding paths in graph from source to destination
int Graph<T>::findpaths(vector<vector<int> > &g, int src, int dst) { //v = adjVec.size(), create a queue which stores the paths
    std::queue<vector<int> > q; // path vector to store the current path
    vector<int> path;
    path.push_back(src);
    q.push(path);
    while (!q.empty()) {
        path = q.front();
        q.pop();
        int last = path[path.size() - 1]; // if last vertex is the desired destination, then print the path
        if (last == dst) {
            printpath(path);
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
void Graph<T>::girvanNewman2() {
    for (int i = 0; i < adjVec.size(); i++) {
        //vector<int> pathSizeVertex;
        for (int j = 0; j < adjVec.size(); j++) {
            girvanDFS(i,j);
            //cout << "finding path in girvan: " << findpaths(adjVec, i, j, adjVec.size()) << " @" << endl;
        }
        //pathSizeVertex.clear();
        //cout << endl << endl;
    }
}


template<class T>
void Graph<T>::girvanDFS(int baap, int destination) {
    vector<int> s;//stack to store the ordering of the elements in the path
    vector<int> g[adjVec.size()];//it stores the adjacent elements of each nodes
    bool visited[adjVec.size()];//it will prevent us from the cycles

    if (baap == destination)//if we found our destination
    {
        //print all the nodes from lower stack to top stack
        for (int i = 0; i < s.size(); i++){
            printf("%d ", s[i]);
        }

        printf("%d\n", destination);
        return;
    }

    visited[baap] = true;
    s.push_back(baap);//pushing the elements while we explore them

    for (int i = 0; i < g[baap].size(); i++) {
        int beta = g[baap][i];
        if (!visited[beta]) {
            girvanDFS(beta, destination);
        }
    }
    visited[baap] = false;
    s.pop_back();//poping the elements while we go out from it
}

template<class T>
void Graph<T>::printShortestDistanceG(int s, int dest) {
    //cout << endl << "----------------------------------------" << endl;
    // predecessor[i] contains parent of i and dist[] stores distance of i from source
    int pred[adjVec.size()], dist[adjVec.size()];

    if (!BFSforGirvan(s, dest, pred, dist)) {
        cout << "Given source and destination cannot be connected";
        return;
    }
}

template<class T>
bool Graph<T>::BFSforGirvan(int src, int dest, int parent[], int distanceFromParent[]) {
    //queue for BFS

    list<int> queue;

    //checks if nodes are visited
    bool visited[adjVec.size()];
    int shortestDistance;
    int parentsOfChildren[adjVec.size()];

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
                if((adjVec[u][i] == dest) && (distanceFromParent[adjVec[u][i]] <= shortestDistance)) {
                    shortestDistance = distanceFromParent[adjVec[u][i]]; //update shortest distance
                    GparentChildrenCount[u]++; //incr the # of children for a vertex
                    visited[adjVec[u][i]] = false; //reset to false so it can be visited again
                }
            }
        }
    }
    calculateBetweenness(parent);
    return true;
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

template<class T>
void Graph<T>::removeNode(int node) {
    for(int i = 0; i < adjVec.size(); i++){
        if(i == node){
            //adjVec[i].remove();
        }
    }
}
#endif //INC_20S_3353_PA02_GRAPH_H