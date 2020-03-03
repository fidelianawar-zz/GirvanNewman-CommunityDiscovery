//
// Created by Fidelia Nawar on 2/26/20.
//

#include <iostream>
#include <fstream>
#include <ostream>
#include <string>
#include <sstream>
#include <unordered_map>
#include <list>
#include "Graph.h"

using std::cout;
using std::cin;
using std::endl;
using std::string;

void readInputFile(std::basic_string<char> input){
    cout << "inputfile is: " << input << endl;
    std::ifstream networkFile(input);

    if(!networkFile){
        cout << "network file cannot open :(";
    }
    else{
        cout << "network file opened!" << endl;
    }
    int numVertices = 0;
    string chars = "[]";

    // first line
    string line;
    if (std::getline(networkFile, line)) {
        // load into string stream for parsing.
        for(char c : chars){
            line.erase(std::remove(line.begin(), line.end(), '['), line.end());
            line.erase(std::remove(line.begin(), line.end(), ']'), line.end());
        }
        std::istringstream iss(line);
        iss >> numVertices;
    }
    cout << "the number of vertices are: " << numVertices;
    //store each vertice as an edge
    networkFile.close();
}

void createOutputFile(std::basic_string<char> output){

    cout << "outputfile is: " << output << endl;

    std::ofstream outputFile(output);
    outputFile << "Writing this to output file.\n";
    outputFile.close();

}

void performDFS(std::basic_string<char> node){
    cout << "inside DFS: " << node << endl;
}

void performBFS(std::basic_string<char> node){
    cout << "inside BFS: " << node << endl;
}

void girvanNewmanAlgo(){
    cout << "inside girvan algo" << endl;
}

int main(int argc, char* const argv[]) {
    std::ifstream controlFile(argv[1]);
    cout << argc << endl;
    if(!controlFile){
        cout << "control file cannot open";
    }

    string command;
    string input;
    std::unordered_map<string,std::pair<string,string>> mcMap;
    std::unordered_map<string, string> commandMap;


    for (std::string line; std::getline(controlFile, line); ) {

        // inserting the line into a stream that helps us parse the content
        std::stringstream ss(line);
        ss >> command;

        if(command == "mc"){
            string arg1, arg2;
            ss >> arg1 >> arg2;
            mcMap.insert({command,std::make_pair(arg1, arg2)});
            //cout << values.at(0).first << " " << values.at(0).second << endl;
        }
        else if(command == "dc"){
            commandMap.insert({command, ""});
        }
        else{
            while(ss >> input){
                commandMap.insert({command, input});
            }
        }
    }

    for (auto itr = commandMap.begin(); itr != commandMap.end(); ++itr) {
        cout << itr->first << '\t' << itr->second << '\n';
    }

    cout << endl;

    for (auto itr = commandMap.begin(); itr != commandMap.end(); ++itr) {
        if(itr -> first == "or"){
            readInputFile(itr->second);
        }
        else if(itr -> first == "ow"){
            createOutputFile(itr->second);
        }
        else if(itr -> first == "bfs"){
            performBFS(itr->second);
        }
        else if(itr -> first == "dfs"){
            performDFS(itr->second);
        }
        else if(itr -> first == "dc"){
            girvanNewmanAlgo();
        }
    }

    for (auto itr = mcMap.begin(); itr != mcMap.end(); ++itr) {
        cout << endl << itr->first << "\t";
        cout << itr->second.first << "\t" << itr->second.second << "\t";
    }
     cout << endl << endl;


    Graph<int> adjList(5);
    //adjList.addEdge(1, 5);


    return 0;
}
