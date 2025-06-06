# GraphTest

This project implements an undirected, weighted graph in C++ that supports multiple graph operations including:

- Reading from a CSV file
- Querying graph properties (nodes, edges, neighbors, weights)
- Finding shortest paths (unweighted and weighted)
- Extracting connected components under a weight threshold
- Finding the smallest threshold to connect two nodes

---

## 📁 File Structure
.
├── src/
│   ├── Graph.cpp         # Implementation of Graph class
│   ├── Graph.h           # Header file with Graph declarations
│   ├── GraphTest.cpp     # Entry point to test all functions
│   ├── Compare.h         # Custom comparator for Dijkstra
├── example/
│   └── small.csv         # Sample edge list to test the graph
├── Makefile              # Build script

---

## 🛠 How to Compile

Run the following from the root directory:

```bash
make

./GraphTest <csv_path> <test_type>

Example: 
./GraphTest ./example/small.csv shortest_unweighted


Valid Test Types
Description
graph_properties
Outputs node count, edge count, neighbors, and weights
shortest_unweighted
Finds unweighted shortest paths using BFS
shortest_weighted
Finds weighted shortest paths using Dijkstra’s
connected_components
Finds components using DFS under a weight threshold
smallest_threshold
Finds the minimum edge weight to connect two nodes

📄 CSV Format
A,B,0.1
A,C,0.5
B,C,0.1
B,D,0.1
E,F,0.4
F,G,0.5


🎓 Acknowledgments
This project was developed as part of CSE 100 at UC San Diego, with additional refinements and extensions added independently.


👤 Author

Ramon M.
Cognitive Science @ UCSD (Specialization: Machine Learning & AI)