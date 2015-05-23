#pragma once
#include <string>
#include <vector>
#include "Node.h"
#include <deque>
#include <set>
#include <algorithm>
#include <Eigen/dense>
#include <functional>

#ifdef GRAPHDLL_EXPORTS
#define GRAPHDLL_API __declspec(dllexport) 
#else
#define GRAPHDLL_API __declspec(dllimport) 
#endif

using namespace Eigen;
using namespace std;

class Node;

class Graph
{
	// Broad-First-Search algorithm
	// The second parameter should be a functor/pointer to function
	// or a lambda
	GRAPHDLL_API friend void bfs(Graph, function<void(const Node&)> &);
	GRAPHDLL_API friend void recursiveTri(Graph &);
	GRAPHDLL_API friend void dvhop(Graph &);
public:
	GRAPHDLL_API Graph();
	GRAPHDLL_API ~Graph();

	// build the graph using files
	GRAPHDLL_API void buildFromFile(ifstream &pointData, ifstream &edgeData);

	// calculate the distance between nodes
	GRAPHDLL_API void shortestPath_Floyd();

	// calculate the distance from every point to the anchors
	GRAPHDLL_API void shortestPath_Dijest();

	// store all the points of the graph
	vector<Node> points;

private:

	// build up a node with string data
	void buildNode(string);

	// build up edges between nodes
	void buildEdge(string);

	// record the maximum id
	int maxId;

	// distance between every two nodes
	vector<vector<double>> minimumPathLength;
	
};

GRAPHDLL_API Vector2d trilateration(Matrix<double, Dynamic, 2> &a, VectorXd &b);
GRAPHDLL_API Vector2d calculatePoint(Node &node, vector<Node*> anchors);
GRAPHDLL_API void bfs(Graph graph, function<void(const Node&)> &task);
GRAPHDLL_API void recursiveTri(Graph &);
GRAPHDLL_API void dvhop(Graph &);