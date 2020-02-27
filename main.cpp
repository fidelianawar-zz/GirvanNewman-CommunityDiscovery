//
// Created by Fidelia Nawar on 2/26/20.
//

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <unordered_map>

using std::cout;
using std::cin;
using std::endl;
using std::string;
using std::vector;

void readInputFile(std::basic_string<char>* input){
    cout << "inputfile is: " << input << endl;
    std::ifstream networkFile("g1.txt");

    if(!networkFile){
        cout << "network file cannot open :(";
    }
    else{
        cout << "network file opened!";
    }
    networkFile.close();
}

void createOutputFile(std::basic_string<char> output){
    cout << "outputfile is: " << output << endl;
}

void performDFS(std::basic_string<char> node){
    cout << "inside DFS: " << node << endl;
}

void performBFS(std::basic_string<char> node){
    cout << "inside BFS: " << node << endl;
}

int main(int argc, char* const argv[]) {

    std::ifstream controlFile(argv[1]);

    if(!controlFile){
        cout << "control file cannot open";
    }

    string command;
    string input;
    std::vector<string> values;
    std::unordered_map<string, string> commandArgs;

    int i = 0;

    for (std::string line; std::getline(controlFile, line); ) {

        // inserting the line into a stream that helps us parse the content
        std::stringstream ss(line);
        ss >> command;
        if(command == "mc"){
            break;
        }
        while(ss >> input){
            commandArgs.insert({command,input});
        }
        i++;
    }

    for (auto itr = commandArgs.begin(); itr != commandArgs.end(); ++itr) {
        cout << itr->first << '\t' << itr->second << '\n';
    }

    cout << endl;

    for (auto itr = commandArgs.begin(); itr != commandArgs.end(); ++itr) {
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
    }

    return 0;
}
