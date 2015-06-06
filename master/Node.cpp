#include "Node.h"
#include <iomanip>

GRAPHDLL_API Node::Node() {}

GRAPHDLL_API Node::~Node()
{
}

void Node::addNeighbour(Node &p, double dist) {
	
	neighbourDistance.insert (std::pair<int, double>(p.id, dist));
}

bool Node::operator<(Node node) const
{
	return id < node.id;
}

GRAPHDLL_API double Node::getY() const
{
	return y;
}

GRAPHDLL_API int Node::getId() const
{
	return id;
}
 
GRAPHDLL_API double Node::getX() const
{
	return x;
}

GRAPHDLL_API double Node::getCy() const
{
	return cy;
}

GRAPHDLL_API double Node::getCx() const
{
	return cx;
}

// 显示该节点的误差，计算位置和真实位置
GRAPHDLL_API void Node::displayDifference(std::ostream &os)
{
	os << std::setprecision(15);
	os << "point: " << id;
	os << " (" << abs(cx - x) / x * 100 << "%, " << abs(cy - y) / y * 100 << "%)";
	os << " (" << cx << ", " << cy << ")";
	os << " (" << x << ", " << y << ")" << std::endl;
}
