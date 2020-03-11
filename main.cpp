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
    cout << "Inputfile is: " << input << endl;
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

void createOutputFile(std::basic_string<char> output) {

    cout << "Outputfile is: " << output << endl;

    std::ofstream outputFile(output);
    outputFile << "Writing this to output file.\n";
    outputFile.close();

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

    std::vector<std::pair<string, string>> mcMap;
    std::vector<pair<string, string>> commandMap;

    for (std::string line; std::getline(controlFile, line);) {
        std::stringstream ss(line);
        ss >> command;
        cout << command << " ";
        if (command == "mc") {
            string arg1, arg2;
            ss >> arg1 >> arg2;
            mcMap.push_back(std::make_pair(arg1, arg2));
        }
        else {
            if(command != "dc"){
                while (ss >> input) {
                    cout << input << " " << endl;
                    commandMap.push_back(std::make_pair(command, input));
                }
            }
            else{
                commandMap.push_back({command, ""});
            }
        }
    }

    for (unsigned int i = 0; i < commandMap.size(); i++) {
        //cout << commandMap[i].first << " " << commandMap[i].second << endl;
    }

    cout << endl;
    for (auto itr = commandMap.begin(); itr != commandMap.end(); ++itr) {
        if (itr->first == "or") {
            readInputFile(itr->second);
        } else if (itr->first == "ow") {
            createOutputFile(itr->second);
        } else if (itr->first == "bfs") {
            networkGraph.BFS(itr->second);
        } else if (itr->first == "dfs") {
            networkGraph.DFS(itr->second);
        } else if (itr->first == "dc") {
            networkGraph.printAllPaths();
        }
    }

    for (auto itr = mcMap.begin(); itr != mcMap.end(); ++itr) {
        //discoverCommunities(itr->first,itr->second);
        networkGraph.makeConnection(itr->first, itr->second);
    }
    cout << endl;
    return 0;
}
