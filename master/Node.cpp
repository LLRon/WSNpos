#include "Node.h"
#include <iomanip>

Node::Node() {}

Node::Node(int id, double x, double y):id(id), x(x), y(y)
{
}

// copy constructor
Node::Node(const Node &node) {

	id = node.id;
	x = node.x;
	y = node.y;
	cx = node.cx;
	cy = node.cy;
	isAnchor = node.isAnchor;
	neighbourDistance = node.neighbourDistance;
}


Node::~Node()
{
}

void Node::addNeighbour(Node &p, double dist) {
	neighbourDistance[p.id] = dist;
}

bool Node::operator<(Node node) const
{
	return id < node.id;
}

double Node::getY() const
{
	return y;
}

int Node::getId() const
{
	return id;
}
 
double Node::getX() const
{
	return x;
}

// 显示该节点的误差，计算位置和真实位置
void Node::displayDifference(std::ostream &os)
{
	os << setprecision(6);
	os << "diff: (" << abs(cx - x) / x * 100 << "%, " << abs(cy - y) / y * 100 << "%)" << "\n";
	os << "cal:  (" << cx << ", " << cy << ")" << "\n";
	os << "real: (" << x << ", " << y << ")" << endl;
}
