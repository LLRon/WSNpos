#include "Graph.h"
#include <sstream>
#include <fstream>
#include <limits>
#include <queue>

vector<string> getValues(const string &data);

Graph::Graph()
{
}


Graph::~Graph()
{
}

void Graph::buildNode(const string &data)
{
	vector<string> &values = getValues(data);
	Node p;

	p.name = stoi(values[0]);
	p.x = stod(values[1]);
	p.y = stod(values[2]);
	p.cx = p.x;
	p.cy = p.y;
	p.isAnchor = values[3] == "1";

	points.push_back(p);
	// id is it's index
	points.back().id = points.size() - 1;
}

void Graph::buildEdge(const string &data)
{
	vector<string> &values = getValues(data);
	Node &p1 = points[stoi(values[0]) - 1];
	Node &p2 = points[stoi(values[1]) - 1];
	double d = stod(values[2]);

	p1.addNeighbour(p2, d);
	p2.addNeighbour(p1, d);
}

vector<string> getValues(const string &data) {
	istringstream ss(data);
	string value;
	vector<string> values;
	int i = 0;
	while (ss >> value) {
		values.push_back(value);
		++i;
	}
	return values;
}

GRAPHDLL_API void Graph::buildFromFile(ifstream &pointData, ifstream &edgeData) {
	for (std::string line; std::getline(pointData, line); ) {
		buildNode(line);
	}

	for (std::string line; std::getline(edgeData, line);) {
		buildEdge(line);
	}
}

GRAPHDLL_API void Graph::shortestPath_Floyd()
{
	if (minimumPathLength.size() == points.size()) {
		return;
	}

	minimumPathLength.resize(points.size());
	for (auto &p1 : points) {
		minimumPathLength[p1.getId()] = vector<double>(points.size(), numeric_limits<double>::max());;
		// self distance is 0
		minimumPathLength[p1.getId()][p1.getId()] = 0;
	}

	for (auto &p : points)
	{
		for (auto &pair : p.neighbourDistance) {
			int neighbour = pair.first,
				pid = p.getId();
			// the adjacent point has their immediate distance
			minimumPathLength[pid][neighbour] = 1;
			minimumPathLength[neighbour][pid] = 1;
		}
	}

	for (auto &k : points)
	{
		int kid = k.getId();
		for (auto &v : points)
		{
			int vid = v.getId();
			for (auto &w : points)
			{
				int wid = w.getId();
				if (minimumPathLength[vid][wid] > minimumPathLength[vid][kid] + minimumPathLength[kid][wid])
				{
					minimumPathLength[vid][wid] = minimumPathLength[vid][kid] + minimumPathLength[kid][wid];
				}
			}
		}
	}
}


GRAPHDLL_API void Graph::shortestPath_Dijest()
{
	for (auto &p : points) {
		vector<double> dist(points.size(), numeric_limits<double>::max());
		set<int> seen;
		deque<Node*> neighbours;

		neighbours.push_back(&p);

		while (neighbours.size() > 0) {
			Node *current = neighbours.front();
			neighbours.pop_front();

			for (auto &nextNeighbour : current->neighbourDistance) {
				if (nextNeighbour.second + dist[current->getId()] <
					dist[nextNeighbour.first]) {
					dist[nextNeighbour.first] = nextNeighbour.second + dist[current->getId()];
				}
				if (seen.find(nextNeighbour.first) == seen.end()) {
					seen.insert(nextNeighbour.first);
					neighbours.push_back(&points[nextNeighbour.first]);
				}
			}
		}
	}
}

GRAPHDLL_API vector<Node>::size_type Graph::size()
{
	return points.size();
}

GRAPHDLL_API Node& Graph::getPoint(int id)
{
	try {
		return points[id];
	}
	catch (exception e) {
		printf(e.what());
	}
}


GRAPHDLL_API Vector2d trilateration(Matrix<double, Dynamic, 2> &a, VectorXd &b) {

	auto temp = a.transpose() * a;

	Vector2d ret = temp.inverse() * a.transpose() * b;

	return ret;
}


// solving matrix equation: Ax = b
GRAPHDLL_API Vector2d calculatePoint(Node &node, vector<Node*> anchors) {
	Matrix<double, Dynamic, 2> a(anchors.size() - 1, 2);

	double x1, y1, d1;
	auto p1 = *anchors.begin();
	x1 = p1->getX();
	y1 = p1->getY();
	d1 = node.neighbourDistance[p1->getId()];

	// for matrix b
	double fixed = d1 * d1 - x1 * x1 - y1* y1;

	int i = 0;

	for (auto it = anchors.begin(); it != anchors.end(); it++) {
		if (it == anchors.begin()) {
			continue;
		}
		a(i, 0) = 2 * (x1 - (*it)->getX());
		a(i, 1) = 2 * (y1 - (*it)->getY());

		i++;
	}

	VectorXd b(anchors.size() - 1);
	i = 0;

	for (auto it = anchors.begin(); it != anchors.end(); it++) {
		if (it == anchors.begin()) {
			continue;
		}

		double d = node.neighbourDistance[(*it)->getId()];
		auto node = (*it);

		b(i) = d * d - node->getX() * node->getX() -
			node->getY() * node->getY() - fixed;

		i++;
	}

	return trilateration(a, b);
}

GRAPHDLL_API void recursiveTri(Graph &graph) {
	queue<Node *> tobeCal;

	for (auto &p : graph.points) {
		if (!p.isAnchor) {
			tobeCal.push(&p);
		}
	}

	int maxIteration = 320 * 10;

	while (tobeCal.size() != 0 && maxIteration-- != 0) {
		Node *node = tobeCal.front();
		tobeCal.pop();

		vector<Node *> anchors;

		for (auto it = node->neighbourDistance.cbegin();
			it != node->neighbourDistance.cend(); it++) {
			if (graph.points[it->first].isAnchor) {
				anchors.push_back(&graph.points[it->first]);
			}
		}

		if (anchors.size() < 3) {
			tobeCal.push(node);
			continue;
		}

		auto &ret = calculatePoint(*node, anchors);

		node->cx = ret(0);
		node->cy = ret(1);

		node->isAnchor = true;
	}
}

GRAPHDLL_API void dvhop(Graph &graph) {

	vector<Node *> anchors;

	// find out all the landmarks
	for (auto &p : graph.points) {
		if (p.isAnchor) {
			anchors.push_back(&p);
		}
	}

	// calculate the correction of landmarks
	for (auto &anchor : anchors) {
		double distSum = 0,
			hopSum = 0;

		for (auto &landmark : anchors) {

			if (anchor != landmark) {
				double dx = (anchor->getX() - landmark->getX()),
					dy = (anchor->getY() - landmark->getY());

				distSum += sqrtl(dx*dx + dy*dy);
				hopSum += graph.minimumPathLength[anchor->getId()][landmark->getId()];
			}
		}

		anchor->correction = distSum / hopSum;
	}

	// determine the correction of regular point
	// to be its nearest landmark's correction
	for (auto &p : graph.points) {

		if (p.isAnchor) {
			continue;
		}

		// test
		decltype(anchors[0]) min = anchors[0];

		for (auto &anchor : anchors) {
			if (graph.minimumPathLength[p.getId()][anchor->getId()] <
				graph.minimumPathLength[p.getId()][min->getId()]) {
				min = anchor;
			}
		}

		p.correction = min->correction;

		p.neighbourDistance.clear();

		for (auto &anchor : anchors) {
			p.addNeighbour(*anchor, p.correction * graph.minimumPathLength[p.getId()][anchor->getId()]);
		}

		auto ret = calculatePoint(p, anchors);

		p.cx = ret(0);
		p.cy = ret(1);
	}
}