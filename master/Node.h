#pragma once
#include <map>
#include <ostream>

#ifdef GRAPHDLL_EXPORTS
#define GRAPHDLL_API __declspec(dllexport) 
#else
#define GRAPHDLL_API __declspec(dllimport) 
#endif

class Node
{
public:
	GRAPHDLL_API Node();
	GRAPHDLL_API ~Node();

	//  add a neighbor to current point
	void addNeighbour(Node &p, double dist);
	bool operator<(Node) const;

	GRAPHDLL_API int getId() const;
	GRAPHDLL_API double getX() const;
	GRAPHDLL_API double getY() const;
	GRAPHDLL_API double getCx() const;
	GRAPHDLL_API double getCy() const;

	bool isAnchor;
	bool isPseudoAnchor;

	std::map<int, double> neighbourDistance;

	// computed x, y
	double cx;
	double cy;

	// average hop from node to landmark
	double correction;

	// distance to landmarks
	std::map<int, double> distanceToLandmarks;

	GRAPHDLL_API void displayDifference(std::ostream &);
	GRAPHDLL_API double Node::diffPer() const
	{
		return (abs(cx - getX()) / getX() + abs(cy - getY()) / getY()) / 2;
	}
	GRAPHDLL_API double Node::diffDist() const
	{
		return sqrt((cx - getX())*(cx - getX()) + (cy - getY())*(cy - getY()));
	}

private:

	int name;
	// for query
	int id;
	double x;
	double y;
	
	friend class Graph;
};