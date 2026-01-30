#include "application.h"
#include "json.hpp"
#include <iostream>
#include <limits>
#include <map>
#include <queue> // priority_queue
#include <set>
#include <stack>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "dist.h"
#include "graph.h"

using namespace std;
using json = nlohmann::json;
double INF = numeric_limits<double>::max();

class prioritize {
 public:
  bool operator()(const pair<long long, double>& p1,
                  const pair<long long, double>& p2) const {
    return p1.second > p2.second;
  }
};


// Define a reasonable search radius in miles to link buildings to footways
// 0.01 miles is about 52 feet, which is a good heuristic.
void buildGraph(istream &input, graph<long long, double> &g,
                vector<BuildingInfo> &buildings,
                unordered_map<long long, Coordinates> &coords) {
g = graph<long long, double>{};
    buildings.clear();
    coords.clear();

    json j;
    input >> j;

    const double BUILDING_CONNECT_RADIUS = 0.036;  // miles

    // Temporary storage: all node IDs → coordinates (both buildings and footways)
    unordered_map<long long, Coordinates> allCoords;

    // Step 1: Read buildings
    for (const auto& b : j["buildings"]) {
        long long id = b["id"].get<long long>();
        double lat = b["lat"];
        double lon = b["lon"];
        string fullName = b.value("name", "");
        string abbr = b.value("abbr", "");

        Coordinates loc(lat, lon);
        buildings.emplace_back(id, loc, fullName, abbr);

        g.addVertex(id);
        allCoords[id] = loc;  // temporarily store, but will NOT go into final `coords`
    }

    // Step 2: Read all waypoints (footway nodes)
    unordered_set<long long> footwayNodeIDs;
    for (const auto& w : j["waypoints"]) {
        long long id = w["id"].get<long long>();
        double lat = w["lat"];
        double lon = w["lon"];

        Coordinates loc(lat, lon);
        allCoords[id] = loc;
        coords[id] = loc;           // only footway nodes go into final `coords`
        g.addVertex(id);
        footwayNodeIDs.insert(id);
    }

    // Step 3: Add footway edges (consecutive nodes in each way)
    for (const auto& way : j["footways"]) {
        if (!way.is_array() || way.size() < 2) continue;

        for (size_t i = 0; i + 1 < way.size(); ++i) {
            long long u = way[i].get<long long>();
            long long v = way[i + 1].get<long long>();

            if (allCoords.count(u) == 0 || allCoords.count(v) == 0) continue;

            double distance = distBetween2Points(allCoords[u], allCoords[v]);

            g.addEdge(u, v, distance);
            g.addEdge(v, u, distance);  // undirected
        }
    }

    // Step 4: Connect each building to nearby footway nodes
    for (const auto& building : buildings) {
        Coordinates buildingLoc = building.location;
        long long buildingID = building.id;

        for (const auto& [nodeID, nodeLoc] : coords) {  // only footway nodes
            double distance = distBetween2Points(buildingLoc, nodeLoc);
            if (distance <= BUILDING_CONNECT_RADIUS) {
                g.addEdge(buildingID, nodeID, distance);
                g.addEdge(nodeID, buildingID, distance);
            }
        }
    }
                }

BuildingInfo getBuildingInfo(const vector<BuildingInfo> &buildings,
                             const string &query) {
  for (const BuildingInfo &building : buildings) {
    if (building.abbr == query) {
      return building;
    } else if (building.name.find(query) != string::npos) {
      return building;
    }
  }
  BuildingInfo fail;
  fail.id = -1;
  return fail;
}

BuildingInfo getClosestBuilding(const vector<BuildingInfo> &buildings,
                                Coordinates c) {
  double minDestDist = INF;
  BuildingInfo ret = buildings.at(0);
  for (const BuildingInfo &building : buildings) {
    double dist = distBetween2Points(building.location, c);
    if (dist < minDestDist) {
      minDestDist = dist;
      ret = building;
    }
  }
  return ret;
}

vector<long long> dijkstra(const graph<long long, double> &G, long long start,
                           long long target,
                           const set<long long> &ignoreNodes) {

  set <long long> visited;
  unordered_map <long long, double> dist;
  unordered_map <long long, long long> pred;
  priority_queue<pair<long long, double>,
               vector<pair<long long, double>>,
               prioritize>
    worklist;
  vector <long long> result;

  vector <long long> vertice_vec = G.getVertices();
  size_t vertices = vertice_vec.size();

  for (size_t i = 0; i<vertices; i++){
    dist[vertice_vec[i]] = INF;
  }

  double start_dist = 0.0;
  dist[start] = start_dist;
  visited.emplace(start);
  worklist.push({start,start_dist});

  double weight;
  bool foundTarget = false;
  while (!worklist.empty()){
    
    long long curr = worklist.top().first;
    double curr_dist = worklist.top().second;
    worklist.pop();

    if (curr==target){
      foundTarget = true;
      break;
    } 

    for (auto n: G.neighbors(curr)){
      if (ignoreNodes.count(n)&&n!=target)  continue;
        
        if (G.getWeight(curr, n, weight)){
          double new_dist = weight + curr_dist;

          if (new_dist<dist[n]){
            dist[n] = new_dist;
            pred[n] = curr;
            worklist.push({n,new_dist});
          }
        }
      
    }
    visited.emplace(curr);
  }

  if (foundTarget){
    long long curr = target;
    
    while (curr!=start){
      result.push_back(curr);
      curr = pred[curr];
    }

    result.push_back(start);
    reverse(result.begin(),result.end());

  }

  return result;
}

double pathLength(const graph<long long, double> &G,
                  const vector<long long> &path) {
  double length = 0.0;
  double weight;
  for (size_t i = 0; i + 1 < path.size(); i++) {
    bool res = G.getWeight(path.at(i), path.at(i + 1), weight);
    if (!res) {
      return -1;
    }
    length += weight;
  }
  return length;
}

void outputPath(const vector<long long> &path) {
  for (size_t i = 0; i < path.size(); i++) {
    cout << path.at(i);
    if (i != path.size() - 1) {
      cout << "->";
    }
  }
  cout << endl;
}

// Honestly this function is just a holdover from an old version of the project
void application(const vector<BuildingInfo> &buildings,
                 const graph<long long, double> &G) {
  string person1Building, person2Building;

  set<long long> buildingNodes;
  for (const auto &building : buildings) {
    buildingNodes.insert(building.id);
  }

  cout << endl;
  cout << "Enter person 1's building (partial name or abbreviation), or #> ";
  getline(cin, person1Building);

  while (person1Building != "#") {
    cout << "Enter person 2's building (partial name or abbreviation)> ";
    getline(cin, person2Building);

    // Look up buildings by query
    BuildingInfo p1 = getBuildingInfo(buildings, person1Building);
    BuildingInfo p2 = getBuildingInfo(buildings, person2Building);
    Coordinates P1Coords, P2Coords;
    string P1Name, P2Name;

    if (p1.id == -1) {
      cout << "Person 1's building not found" << endl;
    } else if (p2.id == -1) {
      cout << "Person 2's building not found" << endl;
    } else {
      cout << endl;
      cout << "Person 1's point:" << endl;
      cout << " " << p1.name << endl;
      cout << " " << p1.id << endl;
      cout << " (" << p1.location.lat << ", " << p1.location.lon << ")" << endl;
      cout << "Person 2's point:" << endl;
      cout << " " << p2.name << endl;
      cout << " " << p2.id << endl;
      cout << " (" << p2.location.lon << ", " << p2.location.lon << ")" << endl;

      Coordinates centerCoords = centerBetween2Points(p1.location, p2.location);
      BuildingInfo dest = getClosestBuilding(buildings, centerCoords);

      cout << "Destination Building:" << endl;
      cout << " " << dest.name << endl;
      cout << " " << dest.id << endl;
      cout << " (" << dest.location.lat << ", " << dest.location.lon << ")"
           << endl;

      vector<long long> P1Path = dijkstra(G, p1.id, dest.id, buildingNodes);
      vector<long long> P2Path = dijkstra(G, p2.id, dest.id, buildingNodes);

      // This should NEVER happen with how the graph is built
      if (P1Path.empty() || P2Path.empty()) {
        cout << endl;
        cout << "At least one person was unable to reach the destination "
                "building. Is an edge missing?"
             << endl;
        cout << endl;
      } else {
        cout << endl;
        cout << "Person 1's distance to dest: " << pathLength(G, P1Path);
        cout << " miles" << endl;
        cout << "Path: ";
        outputPath(P1Path);
        cout << endl;
        cout << "Person 2's distance to dest: " << pathLength(G, P2Path);
        cout << " miles" << endl;
        cout << "Path: ";
        outputPath(P2Path);
      }
    }

    //
    // another navigation?
    //
    cout << endl;
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, person1Building);
  }
}
