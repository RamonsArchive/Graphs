#include <vector>
#include <string>
#include <utility> // for std::pair
#include <tuple>   // for std::tuple
#include <queue>
#include <set>     // or unordered_set if you want hash-based
#include <limits>  
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <stack>
#include <functional>
#include "Graph.h"
#include "Compare.h"

using namespace std;


void Graph::addEdge(string u, string v, double w) {
    adjList[u].push_back({v, w});
    adjList[v].push_back({u, w});
}

Graph::Graph(const char* const & edgelist_csv_fn) {
    std::ifstream file(edgelist_csv_fn);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open the file " << edgelist_csv_fn << std::endl;
        return;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string u, v, w_str;

        if (std::getline(ss, u, ',') && std::getline(ss, v, ',') && std::getline(ss, w_str)) {
            double w = std::stod(w_str);
            addEdge(u, v, w);
        }
    }
}

unsigned int Graph::num_nodes() {
    return adjList.size();
}

vector<string> Graph::nodes() {
    vector<string> labels;
    for (const auto & pair : adjList) {
        string label = pair.first;
        labels.push_back(label);
    }
    return labels;
}

unsigned int Graph::num_edges() {
    int numEdges = 0;
    for (const auto & pair : adjList ) {
        numEdges += pair.second.size();
    }
    return numEdges / 2;
}

unsigned int Graph::num_neighbors(string const & node_label) {
    if (adjList.find(node_label) == adjList.end()) {
        std::cerr << "No neighbors for " << node_label << endl;
        return -1;
    }
    return adjList[node_label].size();
}

double Graph::edge_weight(string const & u_label, string const & v_label) {
    if (adjList.find(u_label) == adjList.end()) {
        std::cerr << "No outgoing edges from node " << u_label << endl; 
        return -1;
    }
    vector<pair<string, double>>& neighbors = adjList[u_label];

    for (const auto & pair : neighbors) {
        if (pair.first == v_label) {
            return pair.second;
        }
    }
    return -1;
}

vector<string> Graph::neighbors(string const & node_label) {
    vector<string> neighborNodes;

    if (adjList.find(node_label) == adjList.end()) {
        std::cerr << "No neighbors for " << node_label << endl;
        return neighborNodes;
    }

    vector<pair<string, double>> neighborList = adjList[node_label];
    for (const auto & v : neighborList) {
        neighborNodes.push_back(v.first);
    }
    return neighborNodes;
}

vector<string> Graph::shortest_path_unweighted(string const & start_label, string const & end_label) {
    vector<string> shortest_path;
    if (start_label == end_label) {
        shortest_path.push_back(start_label);
        return shortest_path;
    }

    if (adjList.find(start_label) == adjList.end()) {
        cerr << "Couldnt find start node: " << start_label << endl;
        return shortest_path;
    }

    queue<string> q;
    unordered_map<string, string> parent;
    set<string> visited;

    q.push(start_label);
    visited.insert(start_label);

    while (!q.empty()) {
        string curr = q.front();
        q.pop();

        for (const auto& neighbor : adjList[curr]) {
            const string& next = neighbor.first;
            if (visited.count(next)) {
                continue;
            }

            visited.insert(next);
            parent[next] = curr;
            q.push(next);
            if (next == end_label) {
                string trace = end_label;
                while (trace != start_label) {
                    shortest_path.push_back(trace);
                    trace = parent[trace];
                }
                shortest_path.push_back(start_label);
                reverse(shortest_path.begin(), shortest_path.end());
                return shortest_path;
            }
        }
    }
    return shortest_path;
}

vector<tuple<string,string,double>> Graph::shortest_path_weighted(string const & start_label, string const & end_label) {
    vector<tuple<string, string, double>> shortest_path;
     if (adjList.find(start_label) == adjList.end()) {
        cerr << "No start_label found: " << start_label << endl;
        return shortest_path;
    }

    if (start_label == end_label) {
        shortest_path.push_back({start_label, end_label, -1});
        return shortest_path;
    }
   
    unordered_map<string, double> dist;
    unordered_map<string, tuple<string, double>> prev;
    set<string> visited;

    for (const auto& [node, _] : adjList) {
        dist[node] = numeric_limits<double>::infinity();
    }

    dist[start_label] = 0.0;
    priority_queue<tuple<string, string, double>, vector<tuple<string, string, double>>, Compare> pq;

    pq.push({start_label, start_label, 0.0});
    while (!pq.empty()) {
        auto [u, v, cost] = pq.top();
        pq.pop();

        if (visited.count(v)) {
            continue;
        }
        visited.insert(v);

        if (v != start_label) {
            prev[v] = {u, cost};
        }

        if (v == end_label) {
            break;
        }

        for (const auto & [neighbor, weight] : adjList[v]) {
            if (visited.count(neighbor)) {
                continue;
            }
            double new_dist = dist[v] + weight;
            if (new_dist < dist[neighbor]) {
                dist[neighbor] = new_dist;
                pq.push({v, neighbor, weight});
            }
        }
    }

    if (!prev.count(end_label)) {
        return shortest_path;
    }

    string curr = end_label;
    vector<tuple<string, string, double>> reversed_path;
    while (curr != start_label) {
        auto [parent, weight] = prev[curr];
        reversed_path.push_back({parent, curr, weight});
        curr = parent;
    }

    reverse(reversed_path.begin(), reversed_path.end());
    return reversed_path;
}

vector<vector<string>> Graph::connected_components(double const & threshold) {
    vector<vector<string>> components;
    set<string> visited;

    for (const auto & [node, _] : adjList) {
        if (visited.count(node)) {
            continue;
        }

        vector<string> component;
        stack<string> stk;
        stk.push(node);
        visited.insert(node);

        while (!stk.empty()) {
            string curr = stk.top();
            stk.pop();
            component.push_back(curr);
            
            for (const auto & [neighbor, weight] : adjList[curr]) {
                if (weight <= threshold && !visited.count(neighbor)) {
                    stk.push(neighbor);
                    visited.insert(neighbor);
                }
            }
        }

        components.push_back(component);
    }
    return components;
}

double Graph::smallest_connecting_threshold(string const & start_label, string const & end_label) {
    if (start_label == end_label) {
        return 0;
    }

    vector<tuple<double, string, string>> edges;
    for (const auto & [u, neighbors] : adjList) {
        for (const auto & [v, cost] : neighbors) {
            edges.emplace_back(cost, u, v);
        }
    }

    sort(edges.begin(), edges.end());
    unordered_map<string, string> parent;
    unordered_map<string, int> size;

    function<string(string)> find = [&](string x) {
        if (parent.find(x) == parent.end()) {
            parent[x] = x;
            size[x] = 1;
        }
        if (parent[x] != x) {
            parent[x] = find(parent[x]); // path compression
        }
        return parent[x];
    };

    auto unite = [&](string a, string b) {
        string rootA = find(a);
        string rootB = find(b);
        if (rootA == rootB) {
            return;
        }

        if (size[rootA] < size[rootB]) {
            parent[rootA] = rootB;
            size[rootB] += size[rootA];
        } else {
            parent[rootB] = rootA;
            size[rootA] += size[rootB];
        }
    };

    for (const auto& [w, u, v] : edges) {
        unite(u, v);
        if (find(start_label) == find(end_label)) {
            return w;
        }
    }
    return -1;

}
