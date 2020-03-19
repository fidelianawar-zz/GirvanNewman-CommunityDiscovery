//
// Created by Fidelia Nawar on 2/26/20.
//

#include <iostream>
#include <fstream>
#include <algorithm>
#include <ostream>
#include <string>
#include <sstream>
#include "Graph.h"

using std::cout;
using std::cin;
using std::endl;
using std::string;
using std::pair;

Graph<string> networkGraph;

void readInputFile(std::basic_string<char> input) {
    //cout << "Input is: " << input << endl;
    std::ifstream networkFile(input);

    if (!networkFile) {
        cout << "Network file cannot open :(";
        exit(5);
    } else {
        cout << "Network file opened!" << endl;
    }

    int numVertices = 0;
    string chars = "[]";

    // first line
    string line;
    if (std::getline(networkFile, line)) {
        for (char c : chars) {
            line.erase(std::remove(line.begin(), line.end(), c), line.end());
        }
        std::istringstream iss(line);
        iss >> numVertices;
    }

    string vertex;
    vector<pair<string, string>> adjPair;

    for (int i = 0; i < numVertices; i++) {
        getline(networkFile, vertex);
        networkGraph.hashVertex(vertex);
    }
    networkGraph.createAdj(numVertices);
    networkFile.ignore(256, '\n');
    string edges;

    while (std::getline(networkFile, edges)) {
        string u, v, newEdge;
        edges.erase(std::remove(edges.begin(), edges.end(), '-'), edges.end());
        std::stringstream ss(edges);
        ss >> u >> v;
        adjPair.push_back(make_pair(u, v));
    }

    cout << "Populating Adjacency List" << endl;
    for(unsigned int i = 0; i < adjPair.size(); i++){
        networkGraph.populateAdj(adjPair[i].first, adjPair[i].second);
    }

    networkGraph.displayAdjVec();
    networkFile.close();
}

void discoverCommunities(std::basic_string<char> s, std::basic_string<char> d) {
    networkGraph.getAllPaths(s, d);
}

int main(int argc, char *const argv[]) {
    std::ifstream controlFile(argv[1]);
    cout << argc << endl;
    if (!controlFile) {
        cout << "Control file cannot open";
    }

    string command, input;

    std::vector<std::pair<string, string>> makeConnectionsVec;
    std::vector<pair<string, string>> commandsVec;

    for (std::string line; std::getline(controlFile, line);) {
        std::stringstream ss(line);
        ss >> command;
        //cout << command << " ";
        if (command == "mc") {
            string arg1, arg2;
            ss >> arg1 >> arg2;
            makeConnectionsVec.push_back(std::make_pair(arg1, arg2));
        }
        else {
            if(command != "dc"){
                while (ss >> input) {
                    //cout << input << " " << endl;
                    commandsVec.push_back(std::make_pair(command, input));
                }
            }
            else{
                commandsVec.push_back({command, ""});
            }
        }
    }

    //cout << endl;

    std::ofstream outputFile;

    for (auto itr = commandsVec.begin(); itr != commandsVec.end(); ++itr) {
        if (itr->first == "or") {
            readInputFile(itr->second);
        } else if (itr->first == "ow") {
            outputFile.open(itr->second);
            if(!outputFile){ cout << "welp shit" << endl; }
            //networkGraph.writeOutput(outputFile);
        } else if (itr->first == "bfs") {
            networkGraph.BFS(itr->second, outputFile);
        } else if (itr->first == "dfs") {
            networkGraph.DFS(itr->second, outputFile);
        } else if (itr->first == "dc") {
            networkGraph.girvanNewman();
        }
    }

    for (auto itr = makeConnectionsVec.begin(); itr != makeConnectionsVec.end(); ++itr) {
        //discoverCommunities(itr->first,itr->second);
        networkGraph.makeConnection(itr->first, itr->second, outputFile);
    }
    outputFile.flush();
    outputFile.close();
    return 0;
}
