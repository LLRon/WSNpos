#include "Graph.h"
#include <sstream>
#include <fstream>
#include <limits>
#include <queue>

vector<string> getValues(string data);

Graph::Graph() : maxId(0)
{
}


Graph::~Graph()
{
}

void Graph::buildNode(string data)
{
	vector<string> &values = getValues(data);
	Node p;

	p.id = stoi(values[0]);
	p.x = stod(values[1]);
	p.y = stod(values[2]);
	p.cx = p.x;
	p.cy = p.y;
	p.isAnchor = values[3] == "1";

	// record max id
	maxId = max(maxId, p.id);

	points.push_back(p);
}

void Graph::buildEdge(string data)
{
	vector<string> &values = getValues(data);
	Node &p1 = points[stoi(values[0]) - 1];
	Node &p2 = points[stoi(values[1]) - 1];
	double d = stod(values[2]);

	p1.addNeighbour(p2, d);
	p2.addNeighbour(p1, d);
}

vector<string> getValues(string data) {
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

void Graph::buildFromFile(ifstream &pointData, ifstream &edgeData) {
	for (std::string line; std::getline(pointData, line); ) {
		buildNode(line);
	}

	for (std::string line; std::getline(edgeData, line);) {
		buildEdge(line);
	}
}

void Graph::shortestPath_Floyd()
{
	minimumPathLength.resize(maxId + 1);
	for (auto &p1 : points) {
		minimumPathLength[p1.getId()] = vector<double>(maxId + 1, numeric_limits<double>::max());;
		// self distance is 0
		minimumPathLength[p1.getId()][p1.getId()] = 0;
	}

	int v, w, k;
	int graphSize = points.size();

	for (auto &p : points)
	{
		for (auto &pair : p.distance) {
			int neighbour = pair.first->getId(),
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


void Graph::shortestPath_Dijest()
{
	for (auto &p : points) {
		vector<double> dist(maxId + 1, numeric_limits<double>::max());
		set<Node*> seen;
		deque<Node*> neighbours;

		neighbours.push_back(&p);

		while (neighbours.size() > 0) {
			Node *current = neighbours.front();
			neighbours.pop_front();

			for (auto &nextNeighbour : current->distance) {
				if (nextNeighbour.second + dist[current->getId()] <
					dist[nextNeighbour.first->getId()]) {
					dist[nextNeighbour.first->getId()] = nextNeighbour.second + dist[current->getId()];
				}
				if (seen.find(nextNeighbour.first) == seen.end()) {
					seen.insert(nextNeighbour.first);
					neighbours.push_back(nextNeighbour.first);
				}
			}
		}
	}
}


Vector2d trilateration(Matrix<double, Dynamic, 2> &a, VectorXd &b) {

	auto temp = a.transpose() * a;

	Vector2d ret = temp.inverse() * a.transpose() * b;

	return ret;
}


// solving matrix equation: Ax = b
Vector2d calculatePoint(Node &node, vector<Node*> anchors) {
	Matrix<double, Dynamic, 2> a(anchors.size() - 1, 2);

	double x1, y1, d1;
	auto p1 = *anchors.begin();
	x1 = p1->getX();
	y1 = p1->getY();
	d1 = node.distance.at(p1);

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

		double d = node.distance.at(*it);
		auto node = (*it);

		b(i) = d * d - node->getX() * node->getX() -
			node->getY() * node->getY() - fixed;

		i++;
	}

	return trilateration(a, b);
}

void recursiveTri(Graph &graph) {
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

		for (auto it = node->distance.cbegin();
			it != node->distance.cend(); it++) {
			if (it->first->isAnchor) {
				anchors.push_back(it->first);
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

void dvhop(Graph &graph) {

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

		p.distance.clear();

		for (auto &anchor : anchors) {
			p.addNeighbour(*anchor, p.correction * graph.minimumPathLength[p.getId()][anchor->getId()]);
		}

		auto ret = calculatePoint(p, anchors);

		p.cx = ret(0);
		p.cy = ret(1);
	}
}

void bfs(Graph &graph, function<void(const Node&)> &task) {

	deque<const Node*> stack;
	set < const Node*> unseen;

	for (auto itor = graph.points.cbegin(); itor != graph.points.cend(); itor++) {
		unseen.insert(&(*itor));
	}

	const Node *current;

	while (unseen.size() > 0) {
		stack.push_back(*unseen.begin());
		unseen.erase(unseen.begin());
		while (stack.size() > 0) {
			current = stack.front();
			stack.pop_front();

			task(*current);

			for (auto itor = current->distance.cbegin(); itor != current->distance.cend(); itor++) {
				set<const Node*>::iterator found = find_if(unseen.cbegin(),
					unseen.cend(),
					[itor](const Node* node){return node->getId() == itor->first->getId(); }
				);

				if (found != unseen.cend()) {
					stack.push_back(itor->first);
					unseen.erase(found);
				}
			}
		}
	}
}