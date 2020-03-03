//
// Created by Fidelia Nawar on 2/27/20.
//

#ifndef INC_20S_3353_PA02_GRAPH_H
#define INC_20S_3353_PA02_GRAPH_H
#include <iostream>
#include <list>
#include <map>

using namespace std;

template<class T>
class Graph {
    int numVertices;

    list<list<T>> adjLists;
    list<T> innerList;

    typename list<list<T>>::iterator *vertexIterator;

    std::map<T, std::list<T>> graphMap;

public:
    Graph(int V);

    void addVertex(T&);
    void addEdge(T src, T dest);
    void displayGraph(int);
};

template<class T>
Graph<T>::Graph(int vertices) {
    numVertices = vertices;
}

//template<class T>
//void Graph<T>::addVertex(const T &v) {
//
//    /*
//    * Access first list in adjList: auto it1 = next(adjlist.begin(), 0);
//    * store it in temp list: list<T> temp = *it1;
//    * access the first element in the first list
//    * auto it2 = next(temp.begin(), 0) if you do *it2 <- this will be a type of T
//    */
//
//}

//typename list<list<T>>::iterator bigIt;
//typename list<T>::iterator littleIt;
//typename list<T>::iterator innerListIterator;

template<class T>
void Graph<T>::addVertex(T& v) {
    list<T> tempList;
    tempList.push_back(v);
    adjLists.push_back(tempList);
    cout << "the size of adj list is: " << adjLists.size() << endl;
}
template<class T>
void Graph<T>::addEdge(T src, T dest){ //src: vertex, dest: edge to be added
    list<T> tempList;
    if (adjLists.empty()) { //adding first element to adjList
        innerList.push_back(src); //innerList is public/global list<T> object
        innerList.push_back(dest);
        adjLists.push_back(innerList); //adjList is public/global list<list<T>> object
    } else {
        //iterate through entire adjList
        for (auto bigIt = adjLists.begin(); bigIt != adjLists.end(); bigIt++) {
            //create tempList for each sublist in adjList
            tempList = *bigIt;
            //grab first element of tempList
            auto val = next(tempList.begin(), 0);
            //check if first element == 'src' parameter
            if (*val == src) {
                //iterate through rest of tempList to make sure 'dest' doesn't already exist
                for (auto innerListIterator = next(tempList.begin(), 1);
                     innerListIterator != tempList.end(); innerListIterator++) {
                    if (*innerListIterator == dest) {
                        cout << "this value already exists for this vertex";
                        return; //exit out of function
                    }
                }
                //if 'dest' doesnt exist, BUT vertex does, push to tempList
                tempList.push_back(dest);
                innerList = tempList;
            }
        }
        //push back inner list to larger adjList
        adjLists.push_back(innerList);
//        if (vertexExists == false) { //first element (vertex) is not in tempList
//            //if src does not exist, create and pushback the vertex to innerList
//            addVertex(src);
//        }
    }
    cout << adjLists.size();
}

//    for(littleIt = tempList.begin(); littleIt != tempList.end(); littleIt++){
//        cout << *littleIt << " ";
//    }
//
//    list<T> tempList;
//    if(adjLists.empty()){ //adding first element to adjList
//        innerList.push_back(src); //innerList is public/global list<T> object
//        innerList.push_back(dest);
//        adjLists.push_back(innerList); //adjList is public/global list<list<T>> object
//    }
//    else{
//        //iterate through entire adjList
//        for (auto bigIt = adjLists.begin(); bigIt != adjLists.end(); bigIt++) {
//            //create tempList for each sublist in adjList
//            tempList = *bigIt;
//            //iterate through temp list
//            for (auto littleIt = tempList.begin(); littleIt != next(tempList.begin()); littleIt++) {
//                //grab first element of tempList
//                auto val = next(tempList.begin(),0);
//                //check if first element == 'src' parameter
//                if(*val == src){
//                    //iterate through rest of tempList to make sure 'dest' doesn't already exist
//                    for(auto innerListIterator = next(tempList.begin(),1); innerListIterator != tempList.end(); innerListIterator++){
//                        if(*innerListIterator == dest){
//                            cout << "this value already exists for this vertex";
//                            return; //exit out of function
//                        }
//                    }
//                    //if 'dest' doesnt exist, push to tempList
//                    tempList.push_back(dest);
//                    innerList = tempList;
//                }
//                else if(*val != src){
//                    //if src does not exist, create and pushback the vertex to innerList
//                    innerList.push_back(src);
//                }
//            }
//            //push back inner list to larger adjList
//            adjLists.push_back(innerList);
//        }
//    }




//adjLists.insert(src,innerList);
//    for(auto it = next(adjLists->begin(),0); it != adjLists->end(); it++){
//
//    }


//    auto bigIt = next(adjLists->begin(), src);
//        list<T> temp = *bigIt;
//        cout << *bigIt;

//        if(bigIt == NULL){
//            list<T> newList = src;
//            adjLists->push_back(src);
//        }
//        list<T> temp = *bigIt;
//        for(auto it = temp.begin(); it != temp.end(); it++){
//
//        }


//    auto bigIterator = next(adjLists->begin(),0);
//    list<T> temp = *bigIterator;
//    auto littleIterator = next(temp->begin(),0);
//    cout << *littleIterator;
//
//
//
//    for(auto iterator = next(adjLists->begin()); iterator != adjLists->end(); iterator++){
//        if(vertex == src){
//            cout << "vertex";
//        }
//    }
//adjLists[src].push_front(dest);



template<class T>
// A utility function to print the adjacency list representation of graph
void Graph<T>::displayGraph(int v){
    for(int i = 0; i < v; i++){
        cout << i << "---->";
        //list<T> :: iterator it;
        for(auto it  = adjLists[i].begin(); it != adjLists[i].end(); ++it) {
            cout << *it << " ";
        }
        cout << endl;
    }
}

#endif //INC_20S_3353_PA02_GRAPH_H