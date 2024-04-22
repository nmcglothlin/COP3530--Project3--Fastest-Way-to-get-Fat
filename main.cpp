# include "src/includes.h"
# include "src/DataProcessing.h"
# include "src/algorithms/Dijkstra.h"
# include "src/algorithms/Bellman.h"

# include <cmath>
# include <chrono>

using namespace bridges;
using namespace std;

string commandList = 
  "  (1) help -- print command list\n"
  "  (2) modify_data -- make changes to the graph data\n"
  "  (3) draw -- draw the graph\n"
  "  (4) find_shortest_path -- find the shortest path between 2 vertexes\n"
  "  (5) quit -- quit the program\n";
string commandInput = ">>>   ";

void modifyData(DataGraph* &dataGraph);
void draw(DataGraph* &dataGraph, int assignmentNumber);
void findShortestPath(DataGraph* &dataGraph);


int main(int argc, char **argv)
{
  int numDraws = 0;

  DataGraph *dataGraph = new DataGraph();
  
  string cmd;
  cout << "Welcome to a Shortest Path Finder!" << endl;
  cout << "Please enter a command: " << endl;
  
  while (true) {
    cout << commandList << endl;
    cout << commandInput;
    cin >> cmd;
    if (cmd == "help" || cmd == "1") {
      cout << commandList << endl;
    }
    else if (cmd == "modify_data" || cmd == "2") {
      modifyData(dataGraph);
    }
    else if (cmd == "draw" || cmd == "3") {
      draw(dataGraph, numDraws);
      numDraws++;
    }
    else if (cmd == "find_shortest_path" || cmd == "shortest_path" || cmd == "4") {
      findShortestPath(dataGraph);
    }
    else if (cmd == "quit" || cmd == "5") {
      break;
    }
    else {
      cout << "Invalid command. Please try again." << endl;
    }
  }

  return 0;
}

void modifyData(DataGraph* &dataGraph) {
  string cmd;
  cout << "==========" << endl;
  cout << "  (1) back -- return to main menu" << endl;
  cout << "  (2) add_vertex -- add a vertex to the graph" << endl;
  cout << "  (3) add_edge -- add an edge to the graph" << endl;
  cout << "  (4) add_random -- add a specified amount of randomized" << endl;
  cout << "                    vertexes and edges to the graph" << endl;
  cout << "  (5) load_from_file -- load a graph from a file" << endl;
  cout << "  (6) save_to_file -- save a graph to a file" << endl;
  cout << "  (7) clear_data -- empty the graph" << endl;
  while (true) {
    cout << commandInput;
    cin >> cmd;
    if (cmd == "back" || cmd == "1") {
      break;
    }
    else if (cmd == "add_vertex" || cmd == "2") {
      string vertCoordsX, vertCoordsY;
      string vertData;
      cout << "Enter the vertex's x-coordinate: ";
      cin >> vertCoordsX;
      cout << "Enter the vertex's y-coordinate: ";
      cin >> vertCoordsY;
      cout << "Eneter the vertex's data: ";
      cin >> vertData;
      Vertex v {dataGraph->numVertexes, {stod(vertCoordsX), stod(vertCoordsY)}, vertData};
      dataGraph->insertVertex(v);
    }
    else if (cmd == "add_edge" || cmd == "3") {
      string v1ID, v2ID;
      string edgeWeight;
      cout << "Enter the first vertex's ID: ";
      cin >> v1ID;
      cout << "Enter the second vertex's ID: ";
      cin >> v2ID;
      cout << "Enter the edge's weight: ";
      cin >> edgeWeight;
      dataGraph->insertEdge(stoi(v1ID), stoi(v2ID), stod(edgeWeight));
    }
    else if (cmd == "add_random" || cmd == "random" || cmd == "4") {
      int numVertexes, numEdges;
      cout << "Enter the number of vertexes: ";
      cin >> numVertexes;
      cout << "Enter the number of edges: ";
      cin >> numEdges;
      dataGraph->addRandom(numVertexes, numEdges);
    }
    else if (cmd == "load_from_file" || cmd == "load" || cmd == "5") {
      DataGraph *newDataGraph = new DataGraph();

      string filename;
      cout << "Enter the filename: ";
      cin >> filename;
      newDataGraph->readFromFile(filename);
      delete dataGraph;
      dataGraph = newDataGraph;
    }
    else if (cmd == "save_to_file" || cmd == "save" || cmd == "6") {
      string filename;
      cout << "Enter the filename: ";
      cin >> filename;
      dataGraph->writeToFile(filename);
    }
    else if (cmd == "clear_data" || cmd == "clear" || cmd == "7") {
      delete dataGraph;
      dataGraph = new DataGraph();
    }
    else if (cmd == "help") {
      cout << "  (1) back -- return to main menu" << endl;
      cout << "  (2) add_vertex -- add a vertex to the graph" << endl;
      cout << "  (3) add_edge -- add an edge to the graph" << endl;
      cout << "  (4) add_random -- add a specified amount of randomized" << endl;
      cout << "                    vertexes and edges to the graph" << endl;
      cout << "  (5) load_from_file -- load a graph from a file" << endl;
      cout << "  (6) save_to_file -- save a graph to a file" << endl;
      cout << "  (7) clear_data -- empty the graph" << endl;
    }
    else {
      cout << "Invalid command. Please try again." << endl;
    }
  }
  cout << "==========" << endl;
  return; 
}
void draw(DataGraph* &dataGraph, int assignmentNumber) {
  cout << "==========" << endl;
  Bridges bridges (
    assignmentNumber, getenv("User ID"), getenv("API Key")
  );
  bridges.setDataStructure(dataGraph->getGraph());
  bridges.setTitle("Graph " + to_string(assignmentNumber));
  bridges.visualize();
  cout << "==========" << endl;
  return;
}
void findShortestPath(DataGraph* &dataGraph) {
  ID startID = -1, endID = -1;
  string algorithm = "Dijkstra";
  string cmd;
  cout << "=========" << endl;
  cout << "  (1) back -- return to main menu" << endl;
  cout << "  (2) change_start_vertex -- change the start vertex" << endl;
  cout << "  (3) change_end_vertex -- change the end vertex" << endl;
  cout << "  (4) find_shortest_path -- find the shortest path between" << endl;
  cout << "                            the start and end vertexes" << endl;
  cout << "  (5) change_alogirthm -- change shortest path algorithm" << endl;
  while (true) {
    cout << commandInput;
    cin >> cmd;
    if (cmd == "back" || cmd == "1") {
      break;
    }
    else if (cmd == "change_start_vertex" || cmd == "start" || cmd == "2") {
      while (true) {
        cout << "Enter the start vertex's ID: ";
        cin >> startID;
        if (startID >= dataGraph->numVertexes || startID < 0) {
          cout << "Invalid vertex ID. Please try again." << endl;
          continue;
        }
        break;
      }
    }
    else if (cmd == "change_end_vertex" || cmd == "end" || cmd == "3") {
      while (true) {
        cout << "Enter the end vertex's ID: ";
        cin >> endID;
        if (endID >= dataGraph->numVertexes || endID < 0) {
          cout << "Invalid vertex ID. Please try again." << endl;
          continue;
        }
        break;
      }
    }
    else if (cmd == "find_shortest_path" || cmd == "shortest_path" || cmd == "4") {
      if (startID == -1 || endID == -1) { 
        cout << "Please set the start and end vertexes first." << endl; 
        continue;
      }
      // calculate shortest paths + time taken
      auto start = std::chrono::high_resolution_clock::now();
      unordered_map<ID, PathVertex> paths;
      if (algorithm == "Dijkstra") { 
        paths = Dijkstra::getShortestPathToVertex(startID, *dataGraph);
      }
      else if (algorithm == "Bellman") {
        paths = Bellman::getShortestPathToVertex(startID, *dataGraph);
      }
      auto end = std::chrono::high_resolution_clock::now();
      // printing
      cout << "Shortest Path: " << endl;
      cout << "  Algorithm: " << algorithm << endl;
      cout << "  Distance: " << paths[endID].distance << endl;
      cout << "  Path: ";
      for (auto pathID : paths[endID].pathIDs) { cout << pathID << " "; }
      cout << paths[endID].thisVertexID << endl;
      cout << "  Time taken: " << 
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms" << endl;
    }
    else if (cmd == "change_alogirthm" || cmd == "algorithm" || cmd == "5") {
      cout << "Available algorithms: " << endl;
      cout << "  -- Dijkstra" << endl;
      cout << "  -- Bellman" << endl;
      while(true) {
        cout << "Enter the algorithm: ";
        cin >> algorithm;
        if (algorithm == "Dijkstra" || algorithm == "Bellman") { break; }  
        
        cout << "Invalid algorithm. Please try again." << endl;
      }
    }
    else if (cmd == "help") {
      cout << "  (1) back -- return to main menu" << endl;
      cout << "  (2) change_start_vertex -- change the start vertex" << endl;
      cout << "  (3) change_end_vertex -- change the end vertex" << endl;
      cout << "  (4) find_shortest_path -- find the shortest path between" << endl;
      cout << "                            the start and end vertexes" << endl;
      cout << "  (5) change_alogirthm -- change shortest path algorithm" << endl;
    }
    else {
      cout << "Invalid command. Please try again." << endl;
    }
  }
  cout << "==========" << endl;
  return;
}