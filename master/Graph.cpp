#include "Graph.h"
#include <sstream>
#include <fstream>
#include <limits>
#include <queue>
#include <iostream>
#include <unsupported/Eigen/MatrixFunctions>

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

	p.isAnchor = values[3] == "1";
	p.isPseudoAnchor = false;

	p.cx = numeric_limits<double>::max();
	p.cy = p.cx;

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

	for (int k = 0; k < points.size(); k++)
	{
		for (int i = 0; i < points.size(); i++)
		{
			for (int j = 0; j < points.size(); j++)
			{
				if (minimumPathLength[i][j] > minimumPathLength[i][k] + minimumPathLength[k][j])
				{
					minimumPathLength[i][j] = minimumPathLength[i][k] + minimumPathLength[k][j];
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
	return (a.transpose() * a).inverse() * a.transpose() * b;
}

// solving matrix equation: Ax = b
GRAPHDLL_API Vector2d calculatePoint(Node &node, vector<Node*> anchors) {
	Matrix<double, Dynamic, 2> a(anchors.size() - 1, 2);

	double x1, y1, d1;
	auto p1 = *anchors.begin();
	x1 = p1->getX();
	y1 = p1->getY();
	d1 = node.distanceToLandmarks.at(p1->getId());

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

		double d = node.distanceToLandmarks.at((*it)->getId());
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
			if (graph.points[it->first].isAnchor || graph.points[it->first].isPseudoAnchor) {
				node->distanceToLandmarks[it->first] = it->second;
				anchors.push_back(&graph.points[it->first]);
			}
		}

		if (node->distanceToLandmarks.size() < 3) {
			tobeCal.push(node);
			continue;
		}

		auto &ret = calculatePoint(*node, anchors);

		node->cx = ret(0);
		node->cy = ret(1);

		node->isPseudoAnchor = true;
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

				distSum += sqrt(dx*dx + dy*dy);
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

		decltype(anchors[0]) min = anchors[0];

		// find the nearest anchor
		for (auto &anchor : anchors) {
			if (graph.minimumPathLength[p.getId()][anchor->getId()] <
				graph.minimumPathLength[p.getId()][min->getId()]) {
				min = anchor;
			}
		}

		p.correction = min->correction;

		for (auto &anchor : anchors) {
			p.distanceToLandmarks[anchor->getId()] = graph.minimumPathLength[p.getId()][anchor->getId()] * p.correction;
		}

		auto ret = calculatePoint(p, anchors);

		p.cx = ret(0);
		p.cy = ret(1);
	}
}

GRAPHDLL_API void pdm(Graph &graph)
{
	vector<Node *> anchors;

	// find out all the landmarks
	for (auto &p : graph.points) {
		if (p.isAnchor) {
			anchors.push_back(&p);
		}
	}

	// P is the proximity matrix and
	// T is the geographic matrix
	MatrixXd P(anchors.size(), anchors.size());
	MatrixXd L(anchors.size(), anchors.size());

	// calculate the correction of landmarks
	for (int i = 0; i < anchors.size(); i++) {
		auto &a1 = anchors[i];
		for (int j = 0; j < anchors.size(); j++) {
			auto &a2 = anchors[j];

			if (a1 != a2) {
				double dx = (a1->getX() - a2->getX()),
					dy = (a1->getY() - a2->getY());

				// geographic distance
				L(i, j) = sqrtl(dx*dx + dy*dy);
				// proximity matrix here assumed to be mini-hop
				P(i, j) = graph.minimumPathLength[a1->getId()][a2->getId()];
			}
			else {
				L(i, j) = 0;
				P(i, j) = 0;
			}
		}
	}

	MatrixXd T = (P.jacobiSvd(ComputeThinU | ComputeThinV).solve(L.transpose())).transpose();

	for (auto &p : graph.points) {
		if (p.isAnchor) {
			continue;
		}

		VectorXd proximity(anchors.size());

		for (int i = 0; i < anchors.size(); i++) {
			proximity(i) = graph.minimumPathLength[p.getId()][anchors[i]->getId()];
		}

		VectorXd geography = T * proximity;

		for (int i = 0; i < anchors.size(); i++) {
			//p.addNeighbour(*anchors[i], geography(i));
			p.distanceToLandmarks[anchors[i]->getId()] = geography(i);
		}

		auto ret = calculatePoint(p, anchors);

		p.cx = ret(0);
		p.cy = ret(1);
	}
}

/*calculate the x(i, j)^2 / (n* m) where x is a nxm matrix*/
double qudraticMean(const MatrixXd &mat) {
	double ret = 0;
	int cols = mat.cols(), rows = mat.rows();
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			ret += mat(i, j) * mat(i, j);
		}
	}
	return ret / cols / rows;
}

MatrixXd doubleCenter(MatrixXd &mat) {
	if (mat.rows() != mat.cols()) {
		throw runtime_error("Can't double center non-square matrix");
	}

	size_t n = mat.rows();
	MatrixXd ret(n, n);

	double means = qudraticMean(mat);

	for (size_t i = 0; i < n; ++i) {
		for (size_t j = 0; j < n; ++j) {
			ret(i, j) = -(mat(i, j)*mat(i, j) - qudraticMean(mat.row(i)) - qudraticMean(mat.col(j)) + means) / 2;
		}
	}

	return ret;
}

GRAPHDLL_API void mds(Graph &graph)
{
	MatrixXd P(graph.minimumPathLength.size(), graph.minimumPathLength.size());

	for (size_t i = 0, n = P.rows(); i < n; ++i) {
		for (size_t j = 0; j < n; ++j) {
			P(i, j) = graph.minimumPathLength[i][j];
		}
	}

	vector<int> anchors;

	// find out all the landmarks
	for (auto &p : graph.points) {
		if (p.isAnchor) {
			anchors.push_back(p.getId());
		}
	}
	cout << "doublecenter start" << endl;
	// double centered matrix
	MatrixXd B = doubleCenter(P);

	cout << "doublecenter end" << endl;

	// singular value decomposition
	cout << "svd start" << endl;
	JacobiSVD<MatrixXd> svd(B, ComputeFullU | ComputeThinV);

	// picking r=2 largest singular value because we are
	// computing in a 2D space
	auto singulars = svd.singularValues();
	MatrixXd U(P.rows(), 2), V(2, 2);
	U.col(0) = svd.matrixU().col(0);
	U.col(1) = svd.matrixU().col(1);

	V << singulars(0), 0, 0, singulars(1);

	cout << "svd end" << endl;

	// compute back to the coordinates matrix
	MatrixXd X = U * V.sqrt();

	// convert relative coordinate to absolute coordinate
	// using transformation computing through anchors
	MatrixXd anchorRel(anchors.size(), 3),
		anchorAbs(anchors.size(), 2);

	for (size_t i = 0; i < anchors.size(); ++i) {
		anchorRel(i, 0) = 1;
		anchorRel(i, 1) = X(anchors[i], 0);
		anchorRel(i, 2) = X(anchors[i], 1);
		anchorAbs(i, 0) = graph.getPoint(anchors[i]).getX();
		anchorAbs(i, 1) = graph.getPoint(anchors[i]).getY();
	}
	cout << "anchor svd start" << endl;
	MatrixXd T = anchorRel.jacobiSvd(ComputeThinU | ComputeThinV).solve(anchorAbs);
	cout << "anchor svd end" << endl;

	VectorXd temp(3);
	for (int i = 0; i < graph.size(); i++) {
		temp << 1,X(i, 0),X(i, 1);
		const MatrixXd &ret = temp.transpose() *T;
		graph.getPoint(i).cx = ret(0);
		graph.getPoint(i).cy = ret(1);
	}
}