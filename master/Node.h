#pragma once
#include <map>
#include "Graph.h"
#include <ostream>

#ifdef GRAPHDLL_EXPORTS
#define GRAPHDLL_API __declspec(dllexport) 
#else
#define GRAPHDLL_API __declspec(dllimport) 
#endif

class GRAPHDLL_API Node
{
public:
	Node();
	Node(int id, double x, double y);
	Node(const Node&);
	~Node();

	//  add a neighbor to current point
	void addNeighbour(Node &p, double dist);
	bool operator<(Node) const;

	int getId() const;
	double getX() const;
	double getY() const;

	bool isAnchor;

	std::map<int, double> neighbourDistance;

	// computed x, y
	double cx;
	double cy;

	// average hop from node to landmark
	double correction;

	void displayDifference(std::ostream &);

private:

	int name;
	// for query
	int id;
	double x;
	double y;

	friend class Graph;
};