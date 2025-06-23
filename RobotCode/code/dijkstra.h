#include "utils.h"
#include "Defines.h"
#include "BluetoothSerial.h"

extern BluetoothSerial DebugSerial;

struct Edge {
  ArraysLengthType targetNode;
  uint8_t weight;  // Using uint16_t for weight (0-65535)
};

struct Node {
  Edge edges[MAX_EDGES];
  ArraysLengthType edgeCount;
};

class Graph {
private:
  Node nodes[MAX_NODES];
  bool nodeExists[MAX_NODES];

public:
  Graph() {
    for (ArraysLengthType i = 0; i < MAX_NODES; i++) {
      nodes[i].edgeCount = 0;
      nodeExists[i] = false;
    }
  }

  // Add an undirected edge between two nodes with full validation
  bool addNode(ArraysLengthType startNode, ArraysLengthType finishNode, uint16_t weight) {
    // Validate node IDs
    if (startNode >= MAX_NODES || finishNode >= MAX_NODES) {
      DebugSerial.print("Error: Node ID out of range (max ");
      DebugSerial.print(MAX_NODES - 1);
      DebugSerial.println(")");
      return false;
    }

    // Check if we can add the first edge
    if (nodes[startNode].edgeCount >= MAX_EDGES) {
      DebugSerial.print("Error: Node ");
      DebugSerial.print(startNode);
      DebugSerial.println(" has no free edge slots");
      return false;
    }

    // Check if we can add the second edge (undirected)
    if (nodes[finishNode].edgeCount >= MAX_EDGES) {
      DebugSerial.print("Error: Node ");
      DebugSerial.print(finishNode);
      DebugSerial.println(" has no free edge slots");
      return false;
    }

    // Check if edge already exists
    for (ArraysLengthType i = 0; i < nodes[startNode].edgeCount; i++) {
      if (nodes[startNode].edges[i].targetNode == finishNode) {
        DebugSerial.print("Error: Edge ");
        DebugSerial.print(startNode);
        DebugSerial.print("->");
        DebugSerial.print(finishNode);
        DebugSerial.println(" already exists");
        return false;
      }
    }

    // Add edge from startNode to finishNode
    nodes[startNode].edges[nodes[startNode].edgeCount] = { finishNode, weight };
    nodes[startNode].edgeCount++;
    nodeExists[startNode] = true;

    // Add edge from finishNode to startNode (undirected graph)
    nodes[finishNode].edges[nodes[finishNode].edgeCount] = { startNode, weight };
    nodes[finishNode].edgeCount++;
    nodeExists[finishNode] = true;
    // printArray(nodeExists, MAX_NODES);
    return true;
  }

  // Calculate shortest distances from startNode using Dijkstra's algorithm
  void calculateDistances(ArraysLengthType startNode, uint16_t distances[], int16_t predecessors[]) {
    // Initialize distances and predecessors
    for (ArraysLengthType i = 0; i < MAX_NODES; i++) {
      distances[i] = UINT16_MAX;  // "Infinity"
      predecessors[i] = -1;       // Undefined
    }
    distances[startNode] = 0;

    // Simple priority queue (we'll use a linear search for min)
    bool visited[MAX_NODES] = { false };

    for (ArraysLengthType count = 0; count < MAX_NODES; count++) {
      // Find the unvisited node with the smallest distance
      ArraysLengthType u = MAX_NODES;
      uint16_t minDist = UINT16_MAX;

      for (ArraysLengthType i = 0; i < MAX_NODES; i++) {
        if (!visited[i] && nodeExists[i] && distances[i] < minDist) {
          minDist = distances[i];
          u = i;
        }
      }

      // If we didn't find any reachable node, break
      if (u == MAX_NODES) break;

      visited[u] = true;

      // Update distances for all adjacent nodes
      for (ArraysLengthType i = 0; i < nodes[u].edgeCount; i++) {
        ArraysLengthType v = nodes[u].edges[i].targetNode;
        uint16_t alt = distances[u] + nodes[u].edges[i].weight;

        if (alt < distances[v]) {
          distances[v] = alt;
          predecessors[v] = u;
        }
      }
    }
  }

  // Reconstruct the path from startNode to finishNode
  // Returns the number of nodes in the path (0 if no path exists)
  // The path array will contain the nodes in reverse order (from finish to start)
  ArraysLengthType returnPath(ArraysLengthType startNode, ArraysLengthType finishNode, int16_t predecessors[], ArraysLengthType path[]) {
    ArraysLengthType pathLength = 0;
    int16_t current = finishNode;

    // Check if there's a path
    if (predecessors[finishNode] == -1 && startNode != finishNode) {
      return 0;  // No path exists
    }

    // Reconstruct the path backwards
    while (current != -1 && pathLength < MAX_NODES) {
      path[pathLength++] = current;
      current = predecessors[current];
    }

    // Reverse the path to get start->finish order
    for (ArraysLengthType i = 0; i < pathLength / 2; i++) {
      ArraysLengthType temp = path[i];
      path[i] = path[pathLength - 1 - i];
      path[pathLength - 1 - i] = temp;
    }

    // Verify the path starts at startNode
    if (pathLength > 0 && path[0] != startNode) {
      return 0;  // No valid path
    }

    return pathLength;
  }

  // Print all nodes and their connections
  void printAllNodes() {
    for (ArraysLengthType i = 0; i < MAX_NODES; i++) {
      if (nodeExists[i]) {
        DebugSerial.print("Node ");
        DebugSerial.print(i);
        DebugSerial.print(" connects to: ");

        for (ArraysLengthType j = 0; j < nodes[i].edgeCount; j++) {
          DebugSerial.print(nodes[i].edges[j].targetNode);
          DebugSerial.print("(w:");
          DebugSerial.print(nodes[i].edges[j].weight);
          DebugSerial.print(")");
          if (j < nodes[i].edgeCount - 1) {
            DebugSerial.print(", ");
          }
        }
        DebugSerial.println();
      }
    }
  }

  // Print shortest distances to all reachable nodes from startNode
  void printShortestDistances(ArraysLengthType startNode) {
    extern ArraysLengthType distances[MAX_NODES];
    extern int16_t predecessors[MAX_NODES];

    calculateDistances(startNode, distances, predecessors);

    DebugSerial.print("Shortest distances from node ");
    DebugSerial.println(startNode);

    for (ArraysLengthType i = 0; i < MAX_NODES; i++) {
      if (nodeExists[i] && distances[i] != UINT16_MAX) {
        DebugSerial.print("To node ");
        DebugSerial.print(i);
        DebugSerial.print(": ");
        DebugSerial.print(distances[i]);

        // Optional: Print path
        extern ArraysLengthType path[MAX_NODES];
        ArraysLengthType pathLength = returnPath(startNode, i, predecessors, path);

        if (pathLength > 0) {
          DebugSerial.print(" (Path: ");
          for (ArraysLengthType j = 0; j < pathLength; j++) {
            DebugSerial.print(path[j]);
            if (j < pathLength - 1) {
              DebugSerial.print("->");
            }
          }
          DebugSerial.print(")");
        }
        DebugSerial.println();
      }
    }
  }
};
